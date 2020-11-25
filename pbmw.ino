#include <Arduino.h>
#include <Adafruit_ADS1015.h>
#include <math.h>
#include <usbmidi.h>

Adafruit_ADS1115 adc;

const unsigned char MIDI_OPCODE_PITCH_BEND = 0B1110;
const unsigned char MIDI_OPCODE_CONTROL_CHANGE = 0B1011;
const unsigned char MIDI_CONTROL_CHANGE_MOD_WHEEL = 1;
const unsigned char MIDI_CHANNEL = 0; // "channel 1"
const unsigned char pb_header = (MIDI_OPCODE_PITCH_BEND << 4) + MIDI_CHANNEL; // opcode | channel
const unsigned char mw_header = (MIDI_OPCODE_CONTROL_CHANGE << 4) + MIDI_CHANNEL; // opcode | channel
const uint16_t jitter_thresh = 5;
const float alpha = 0.95; // {0..1} falloff factor for exponential moving average (higher means older values count less)
const uint16_t ring_buffer_size = 64;
const uint16_t pb_deadzone_radius = 200;
const uint16_t pb_deadzone_lower = 8191 - pb_deadzone_radius;
const uint16_t pb_deadzone_upper = 8191 + pb_deadzone_radius;

long t = 0;

struct ring_buffer {
  size_t max_size;
  size_t size;
  size_t head_index;
  size_t tail_index;
  uint16_t *array;
};

typedef struct ring_buffer ring_buffer;

ring_buffer *ring_buffer_initialize(size_t max_size) {
  ring_buffer *rb = (ring_buffer*)malloc(sizeof(ring_buffer));
  rb->max_size = max_size;
  rb->size = 0;
  rb->head_index = 0;
  rb->tail_index = 0;
  rb->array = (uint16_t*)malloc(max_size * sizeof(uint16_t));
  return rb;
}

void ring_buffer_free(ring_buffer *rb) {
  free(rb->array);
  free(rb);
}

uint16_t ring_buffer_get(ring_buffer *rb, size_t index) {
  return rb->array[(rb->head_index + index) % rb->max_size];
}

void ring_buffer_push(ring_buffer *rb, uint16_t value) {
  size_t new_head_index =
    rb->head_index == 0
      ? rb->max_size - 1
      : rb->head_index - 1;

  size_t new_tail_index =
    rb->size == rb->max_size
      ? (rb->tail_index == 0 ? rb->max_size - 1 : rb->tail_index - 1)
      : rb->tail_index - 1;

  rb->array[new_head_index] = value;
  rb->head_index = new_head_index;
  rb->tail_index = new_tail_index;

  if (rb->size != rb->max_size) { rb->size++; }
}

ring_buffer *pb_samples;
ring_buffer *pb_moving_averages;
ring_buffer *mw_samples;
ring_buffer *mw_moving_averages;

void setup() {
  // Serial.begin(9000);
  pb_samples = ring_buffer_initialize(64);
  mw_samples = ring_buffer_initialize(64);
  pb_moving_averages = ring_buffer_initialize(64);
  mw_moving_averages = ring_buffer_initialize(64);

  adc.setGain(GAIN_TWO);
  adc.begin();
}

// f(xₜ):
//   { when t = 1 }: x₁
//   { when t > 1 }: α * xₜ + (1 - α) * f(x₍ₜ₋₁₎)
//
// where α is a coefficient in {0..1} representing the coefficient of falloff,
// i.e. number of previous samples used, i.e. the cutoff frequency in our
// low-pass filter
//
unsigned int exponential_moving_average(unsigned int x) {
  return(alpha * x + (1 - alpha) * ring_buffer_get(pb_moving_averages, 1));
}

bool is_outside_range(unsigned int x, unsigned int lower, unsigned int upper) {
  return((x < lower) || (x > upper));
}

bool is_within_range(unsigned int x, unsigned int lower, unsigned int upper) {
  return((x >= lower) && (x <= upper));
}

void loop() {
  USBMIDI.poll();

  uint16_t pb_curr = adc.readADC_SingleEnded(0);
  uint16_t mw_curr = adc.readADC_SingleEnded(1);

  // for some reason, the ADS1115 reading has an effective resolution of 15 bits, not 16.
  // https://github.com/adafruit/Adafruit_ADS1X15/pull/9
  //
  // if ADC returns a value over (2**15)-1, we know it's an overflow and should "mean" 0.
  if (pb_curr > 32767) { pb_curr = 0; }
  if (mw_curr > 32767) { mw_curr = 0; }

  pb_curr >>= 1; // 15 bit -> 14 bit
  mw_curr >>= 1; // 15 bit -> 14 bit

  if (t < ring_buffer_size) {
    memset(mw_samples->array, mw_curr, mw_samples->max_size);
    memset(mw_moving_averages->array, mw_curr, mw_moving_averages->max_size);
    t++;
  }

  ring_buffer_push(mw_samples, mw_curr);
  uint16_t mw_ave = exponential_moving_average(mw_curr);
  ring_buffer_push(mw_moving_averages, mw_ave);

  unsigned char mw_lsb = (mw_curr & 0B0000000001111111);
  unsigned char mw_msb = (mw_curr & 0B0011111110000000) >> 7;

  // accomodate for exponential moving average jitter reduction
  if (
    mw_curr != ring_buffer_get(mw_samples, 1) &&
    abs((signed long)mw_curr - (signed long)mw_ave) > jitter_thresh
  ) {
    USBMIDI.write(mw_header);
    USBMIDI.write(MIDI_CONTROL_CHANGE_MOD_WHEEL);
    USBMIDI.write(mw_msb);
    USBMIDI.flush();
  }

  if (pb_curr < (8192 + 100) && pb_curr > (8192 - 100)) {
    pb_curr = 8192;
  }

  ring_buffer_push(pb_samples, pb_curr);
  uint16_t pb_ave = exponential_moving_average(pb_curr);
  ring_buffer_push(pb_moving_averages, pb_ave);

  if (pb_curr != ring_buffer_get(pb_samples, 1)) {
    unsigned char pb_lsb = (pb_curr & 0B0000000001111111);
    unsigned char pb_msb = (pb_curr & 0B0011111110000000) >> 7;

    USBMIDI.write(pb_header);
    USBMIDI.write(pb_lsb);
    USBMIDI.write(pb_msb);
    USBMIDI.flush();
  }
}
