// core hw abstractions

// Pin defs

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
    static uint16_t buffer[ADC_SAMPLES];
    static uint32_t running_sum = 0;
    static uint8_t index = 0;

    running_sum -= buffer[index];
    buffer[index] = adc_read(pin);
    running_sum += buffer[index];
    index = (index + 1) % ADC_SAMPLES;

    return (float)(running_sum / ADC_SAMPLES) / 1023.0f;
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
