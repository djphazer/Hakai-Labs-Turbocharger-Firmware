#pragma once
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Includes
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
//#include <time.h>

static constexpr size_t ADC_SAMPLES = 4;
static constexpr size_t ADC_MAX = 0x3ff;
static constexpr size_t DEBOUNCE_CYCLES = 50; // Number of cycles to wait for stable reading
static constexpr size_t CYCLE_AVG_SAMPLES = 3; // Number of cycles to average

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Constants - Fine Tuning
/////////////////////////////////////////////////////////////////////////////////////////////////////////
constexpr int MAJOR_VERSION_NUMBER = 3;
constexpr int MINOR_VERSION_NUMBER = 1;
constexpr uint16_t STATUS_MAJOR_VERSION_BLINK_DELAY = 300; // Status LED LED blink delay in ms
constexpr uint16_t STATUS_MINOR_VERSION_BLINK_DELAY = 100; // Status LED LED blink delay in ms
constexpr uint16_t MIN_GLITCH_DURATION = 20; // Minimum glitch duration in ms
constexpr uint16_t MAX_GLITCH_DURATION = 200; // Maximum glitch duration in ms
constexpr uint16_t MAX_DELAY_BETWEEN_GLITCHES = 600; // Maximum delay between glitches in ms
constexpr uint16_t MIN_DELAY_BETWEEN_GLITCHES = 1; // Minimum delay between glitches in ms
constexpr uint16_t GAP_LOOPS_PER_MS = 2; // main-loop iterations per ms; reduce to shorten gap, increase to lengthen it
constexpr uint16_t DET_RESEED_THRESHOLD = 0.05f * ADC_MAX; // det knob must move this much (0.0-1.0) to advance seed_salt and re-randomize glitches
constexpr uint16_t DET_POT_DEADBAND = 0.05f * ADC_MAX; // det ADC must change by more than this (0.0-1.0) to update det_value; suppresses hardware noise
constexpr float MIN_INTERNAL_RPM_FREQ_HZ = 1.5328125; // Minimum Internal RPM Frequency in hz (1V/octave at -5V)
constexpr float MAX_INTERNAL_RPM_FREQ_HZ = 1569.6; // Maximum Internal RPM Frequency in hz (1V/octave at +5V)

// --- core hw abstractions ---

// TODO: Pin defs?

// driver functions
namespace hw {
  void init_ports(void) {
    // Configre my inputs for the module
    PORTA.DIRCLR = PIN1_bm; // Potentiometer Turbo Speed Control Input (Internal RPM pot) - 0-5v
    PORTA.DIRCLR = PIN2_bm; // Potentiometer AFR Input - 0-5v
    PORTA.DIRCLR = PIN4_bm; // Potentiometer Detonation Input - 0-5v
    PORTA.DIRCLR = PIN5_bm; // Gate Input 0 or 5v.  Used to send a gate to trigger the output on and off.  When nothing is connected to this input, it will default to 5v and will let the output flow trough constantly.
    PORTA.DIRCLR = PIN6_bm; // Floating Input for random
    PORTB.DIRCLR = PIN3_bm; // Turbo In

    // Enable pull-ups for the specified inputs
    PORTA.PIN1CTRL = PORT_PULLUPEN_bm; // Internal RPM Potentiometer
    PORTA.PIN2CTRL = PORT_PULLUPEN_bm; // AFR Potentiometer
    PORTA.PIN4CTRL = PORT_PULLUPEN_bm; // Detonation Potentiometer
    PORTA.PIN5CTRL = PORT_PULLUPEN_bm; // Gate Input
    PORTA.PIN6CTRL = ~PORT_PULLUPEN_bm; // DISABLE Pull-up Floating Input for random
    PORTB.PIN3CTRL = PORT_PULLUPEN_bm; // Turbo In, this may interfere with the signal, so need to test.

    // Configre the outputs for the module
    PORTA.DIRSET = PIN7_bm; // Detonation Status Out LED
    PORTB.DIRSET = PIN0_bm; // Detonation Injection Output.  Connected to PortB Pin 1 with a diode.
    PORTB.DIRSET = PIN1_bm; // Internal RPM Clock Out (PWM on WO1)
    PORTB.DIRSET = PIN2_bm; // System Status Out LED.  Used for debugging and system info.

    // Initial disable of detonation pins just in case something gets stuck
    PORTB.OUTCLR = PIN0_bm; // Disable Detonation Output just in case
    PORTA.OUTCLR = PIN7_bm; // Disable Detonation LED just in case

    // Enable Pin Change Interrupts
    PORTB.PIN3CTRL = PORT_ISC_RISING_gc; //enable pin interrupts for rising edge for Turbo In. PORT_ISC_RISING_gc or PORT_ISC_BOTHEDGES_gc
    PORTA.PIN5CTRL = PORT_ISC_RISING_gc; //enable pin interrupts for rising edge for Turbo In. PORT_ISC_RISING_gc or PORT_ISC_BOTHEDGES_gc
  }

  inline void set_det_out(const bool on) {
    if (on) {
      PORTA.OUTSET = PIN7_bm; // LED
      PORTB.OUTSET = PIN0_bm; // Injection output
    } else {
      PORTA.OUTCLR = PIN7_bm;
      PORTB.OUTCLR = PIN0_bm;
    }
  }

  void blink_version_led(void) {
    // Major Version Blink
    for (int i = 0; i < MAJOR_VERSION_NUMBER; i++) {
      PORTB.OUTSET = PIN2_bm;
      _delay_ms(STATUS_MAJOR_VERSION_BLINK_DELAY);
      PORTB.OUTCLR = PIN2_bm;
      _delay_ms(STATUS_MAJOR_VERSION_BLINK_DELAY);
    }
    // Minor Version Blink
    for (int i = 0; i < MINOR_VERSION_NUMBER; i++) {
      PORTB.OUTSET = PIN2_bm;
      _delay_ms(STATUS_MINOR_VERSION_BLINK_DELAY);
      PORTB.OUTCLR = PIN2_bm;
      _delay_ms(STATUS_MINOR_VERSION_BLINK_DELAY);
    }
  }

  void adc_init(void) {
    ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV16_gc; // Slower ADC clock
    ADC0.CTRLA = ADC_ENABLE_bm;
    PORTA.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    PORTA.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  }

  uint16_t adc_read(uint8_t channel) {
    ADC0.MUXPOS = channel;
    ADC0.COMMAND = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { }
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    return ADC0.RES;
  }

  // 10-bit ADC reading, smoothed
  // templated so there are separate static buffers for each channel
  template <uint8_t pin>
  uint16_t adc_read_avg() {
    // O(1) running sum: subtract outgoing sample, add incoming sample.
    // Max sum = 1023 * 20 = 20460, well within uint32_t.
    static uint16_t buffer[ADC_SAMPLES] = {0};
    static uint32_t running_sum = 0;
    static uint8_t index = 0;

    running_sum -= buffer[index];
    buffer[index] = adc_read(pin);
    running_sum += buffer[index];
    index = (index + 1) % ADC_SAMPLES;

    const uint16_t avg = running_sum / ADC_SAMPLES;
    return (avg & ~0x3ff) ? 1023 : avg; // clipped at 10 bits
  }

  void init(void) {
    init_ports();
    sei(); // Enable global interrupts
    blink_version_led();
    adc_init();

    // --- Reset and initialize PWM timer
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CNT = 0;
    TCA0.SINGLE.PER = 0xffff; // max period
    TCA0.SINGLE.CMP1 = 0;

    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm;
    // I was experimenting with the 1024 prescaler here, but decided to use 64.
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm; // Set prescaler value to 64
  }
}
