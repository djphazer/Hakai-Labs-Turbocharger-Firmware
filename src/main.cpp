#include "drivers.h"

// Variables for Turbo In signal processing (detonation detection)
volatile long turbo_cycle_count = 0;
uint32_t last_total_cycle_counts[CYCLE_AVG_SAMPLES] = {0}; // Array to store last cycles
volatile long cycle_index = 0; // Index for the circular buffer
volatile uint32_t last_total_cycle_count = 1; // Average of the last cycles (initialized to 1)
volatile bool detonation_clock_mult_status = false;
volatile long detonation_clock_cycle_reset_counter = 1;
volatile uint16_t precomputed_rpm_division = 32;

uint16_t gate_trigger_counter = 0;
uint16_t current_det_value_within_type = 0;
long det_random_number = 0;
long det_glitch_duration = 0;


//RPM Division Variables — pre-computed in main loop, read by ISR

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global System State
/////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t gate_division_value = 8;
bool gate_last_value = true;
bool detonation_armed = false;

uint16_t rpm_value = 0;
uint16_t afr_value = 0;
uint16_t det_value = 0;

uint16_t prev_rpm_value = 0;
uint16_t prev_afr_value = 0;
uint16_t prev_detonation_value = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Calculation Utility Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////
constexpr float get_frequency_from_rpm_value(uint16_t rpm) {
  // 1V/octave exponential scaling: frequency = base_freq * 2^(octaves)
  // rpm is normalized ADC (0-1), which maps to CV voltage range -5V to +5V
  // CV_volts = (rpm * 10.0) - 5.0
  // Frequency doubles for each volt increase (1V/octave)
  // At CV = -5V (rpm = 0): frequency = 4.0875 * 2^0 = 4.0875 Hz
  // At CV = +5V (rpm = 1): frequency = 4.0875 * 2^10 = 4185.6 Hz
  float cv_volts = (float(rpm) * 10.0f / ADC_MAX) - 5.0f;
  return MIN_INTERNAL_RPM_FREQ_HZ * powf(2.0f, cv_volts + 5.0f);
}

constexpr uint32_t get_period_from_frequency(uint16_t frequency) {
  // Prevent division by zero or invalid frequencies
  if (frequency < 0.01) {
    frequency = 0.01; // Set a safe minimum frequency
  }

  // Calculate the period based on the CPU clock and prescaler
  // I was experimenting with the 1024 prescaler here, but decided to use 64.
  uint32_t period = (uint32_t)(F_CPU / (64.0f * frequency)); // For 64 Prescaler

  // Ensure the period fits within a 16-bit timer
  if (period > 65535) {
    period = 65535; // Maximum valid period
  } else if (period < 1) {
    period = 1; // Minimum valid period
  }
  return period;
}

/////////////////////////
// Analog read helpers //
/////////////////////////
// TODO: convert to fixed-point? or do something completely different...
void update_internal_rpm_pot_value(void) {
  uint16_t adc_value = hw::adc_read_avg<ADC_MUXPOS_AIN1_gc>();

  rpm_value = ADC_MAX - adc_value;

  // Pre-compute RPM division for the ISR so it can avoid powf/logf.
  // freq_normalized = log2(MIN * 2^(rpm*10) / MIN) / 10 = rpm_value,
  // so the full 1V/octave frequency normalization reduces to rpm_value itself.
  int arr_idx = (rpm_value * 5) / ADC_MAX;
  if (arr_idx > 5) arr_idx = 5;
  if (arr_idx < 0) arr_idx = 0;
  precomputed_rpm_division = (uint16_t)(32u << arr_idx);
}

void update_afr_pot_value(void) {
  uint16_t adc_value = hw::adc_read_avg<ADC_MUXPOS_AIN2_gc>();
  if (adc_value > 0.9f * ADC_MAX) {
    adc_value = ADC_MAX;
  } else if (adc_value < 0.1f * ADC_MAX) {
    adc_value = 0;
  } else {
    adc_value = (1.25f * adc_value) - 0.125f * ADC_MAX;
  }
  afr_value = ADC_MAX - adc_value;
}

void update_detonation_pot_value(void) {
  uint16_t adc_value = hw::adc_read_avg<ADC_MUXPOS_AIN4_gc>();

  if (adc_value < 0.01f * ADC_MAX) {
    adc_value = 0.01f * ADC_MAX;
  }
  uint16_t new_det = ADC_MAX - adc_value;
  // Only update det_value if the reading has moved more than DET_POT_DEADBAND.
  // This suppresses ADC noise so small hardware drifts do not alter the det state.
  if (new_det > det_value + DET_POT_DEADBAND || new_det < det_value - DET_POT_DEADBAND) {
    det_value = new_det;
  }
}

uint16_t read_rng(void) {
  // TODO: figure out how many usable bits of noise we get from this and normalize to a useful range
  return hw::adc_read(ADC_MUXPOS_AIN6_gc);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// This is the interrupt for the internal or external RPM clock signal.
ISR(PORTB_PORT_vect) {
  // Store the current cycle count in the array
  last_total_cycle_counts[cycle_index] = turbo_cycle_count;

  // Calculate the new average
  uint32_t sum = 0;
  for (int i = 0; i < CYCLE_AVG_SAMPLES; i++) {
    sum += last_total_cycle_counts[i];
  }
  last_total_cycle_count = sum / CYCLE_AVG_SAMPLES;

  // Increment the index for the next cycle count
  cycle_index = (cycle_index + 1) % CYCLE_AVG_SAMPLES; // Circular buffer update

  turbo_cycle_count = 0;

  // Use value pre-computed in main loop — avoids powf/logf inside the ISR.
  // (freq_normalized = log2(MIN*2^(rpm*10)/MIN)/10 = rpm_value, so the full
  //  frequency calculation reduces to a simple multiply; pre-computing it in
  //  update_internal_rpm_pot_value() keeps the ISR lean.)
  if (detonation_clock_cycle_reset_counter < precomputed_rpm_division) {
    detonation_clock_cycle_reset_counter++;
  } else {
    detonation_clock_cycle_reset_counter = 1;
  }

  VPORTB.INTFLAGS = PIN3_bm;
}

// This is the interrupt for the gate signal.
ISR(PORTA_PORT_vect) {
  // Stop the timer, reset the counter to BOTTOM, and restart.
  // In TCA single-slope PWM mode the output is SET HIGH at BOTTOM (CNT=0)
  // by the hardware — no need to set CMP1=PER to force a high output.
  // Setting CMP1=PER would keep the output stuck HIGH permanently because
  // the compare-match (which clears the output) never fires at that value.
  // Simply restarting from CNT=0 gives an immediate HIGH and then the correct
  // duty cycle resumes on the same cycle.
  TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm; // Stop timer
  TCA0.SINGLE.CNT = 0; // Reset to BOTTOM — output goes HIGH
  TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; // Restart timer

  // Every X gates, reset the clock cycle reset counter
  // This affects the detonation logic forcing looping random patterns
  int gate_array_index = (afr_value * 11) / ADC_MAX;

  if (gate_array_index == 11) {
    gate_array_index = 10;
  }

  gate_division_value = (gate_array_index < 6) ? (gate_array_index + 3) : (1 << (gate_array_index - 2));

  if (gate_trigger_counter >= gate_division_value) {
    detonation_clock_cycle_reset_counter = 1;
    gate_trigger_counter = 0;
  }

  gate_trigger_counter++; // This reset might need to be moved up above the X cycle check

  VPORTA.INTFLAGS = PIN5_bm;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Custom CPU Delay Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void custom_delay_us(uint16_t us) {
  while (us--) {
    _delay_us(1);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Detonation Notes
// Types and ID of Detonation Glitches
//  1: Clock Divide every 8 cycles
//  2: Clock Divide every 4 cycles
//  3: Clock Divide every 3 cycles
//  4: Clock Divide every 2 cycles
//  5: Clock Divide every 1 cycles
//  6: Random injection plus oscillating frequencies based on Detonation Potentiometer value
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void process_turbo_cycle(void) {
  // Round-robin ADC reads: one channel per call instead of three.
  // Each channel is still sampled at most ~3x the loop rate through the
  // 20-sample averaging buffer, so response time is effectively unchanged.
  static uint8_t adc_rr = 0;
  switch (adc_rr) {
  case 0:
    update_internal_rpm_pot_value();
    break;
  case 1:
    update_afr_pot_value();
    break;
  default:
    update_detonation_pot_value();
    break;
  }
  adc_rr = (adc_rr >= 2) ? 0 : adc_rr + 1;

  if (det_value > 0.075f * ADC_MAX) {
    detonation_armed = true;
  } else {
    detonation_armed = false;
    hw::set_det_out(false);
    turbo_cycle_count = turbo_cycle_count + 1;
    return;
  }

  int current_position = 100 * turbo_cycle_count / last_total_cycle_count;

  ///////////////////////////////////////////////////////////////////////////////////
  // DETONATION TYPES 1-5
  // Divisor controls how often (every N RPM clock cycles) the det arm window fires.
  //   Type 1 (<=0.2): every 8 cycles
  //   Type 2 (<=0.3): every 4 cycles
  //   Type 3 (<=0.4): every 3 cycles
  //   Type 4 (<=0.5): every 2 cycles
  //   Type 5 (<=0.6): every cycle
  ///////////////////////////////////////////////////////////////////////////////////
  if (det_value <= 0.6f * ADC_MAX) {
    uint8_t det_divisor = (det_value <= 0.2f * ADC_MAX) ? 8
      : (det_value <= 0.3f * ADC_MAX)                   ? 4
      : (det_value <= 0.4f * ADC_MAX)                   ? 3
      : (det_value <= 0.5f * ADC_MAX)                   ? 2
                                                        : 1;
    // Both conditions are evaluated every iteration — no sticky state.
    // Previously the flag was only written inside the counter-match guard, so if
    // a turbo pulse (ISR) arrived while position was inside the [0.5, 0.75] window
    // the flag was left TRUE and the output locked high for the entire next cycle.
    detonation_clock_mult_status = (detonation_clock_cycle_reset_counter % det_divisor == 0)
      && (current_position >= 50 && current_position <= 75);

    ///////////////////////////////////////////////////////////////////////////////////
    // DETONATION TYPE 6 - APC (Atari Punk Console) Mode
    // Fires glitches at random intervals gated by detonation_clock_cycle_reset_counter.
    // Each glitch oscillates the GLITCH_ADD output at a harmonic of the RPM frequency.
    //
    // Sweep types control how the oscillation pitch evolves over the glitch duration:
    //   1 = Flat      : constant frequency throughout
    //   2 = Ascending : delay_us increases each cycle (pitch descends)
    //   3 = Descending: delay_us decreases each cycle (pitch ascends)
    //   4 = Alternate : octave jump up/down every 2 pulses (harmonics capped at 4x RPM)
    //
    // Harmonic selection is biased by knob position so glitch frequencies stay
    // musically related to the current RPM clock:
    //   Low knob (0.0-0.4 of Type 6): picks 1x, 2x, 3x, or 4x RPM
    //   Mid knob (0.4-0.6):           picks 1x through 8x RPM
    //   High knob (0.6-1.0):          picks 1x through 16x RPM
    ///////////////////////////////////////////////////////////////////////////////////
  } else if (det_value > 0.6f * ADC_MAX && det_value <= ADC_MAX) {
    current_det_value_within_type = (det_value - 0.6f * ADC_MAX) * 2; // 0.0 to 1.0 within Type 6

    // --- Non-blocking inter-glitch gap ---
    // When det_gap_countdown > 0 the main loop runs freely (pot reads, TCA0 PERBUF
    // writes, ISR handling) and the glitch trigger is skipped entirely.
    // On each call it decrements by 1; when it reaches 0 the next glitch is allowed.
    static uint16_t det_gap_countdown = 0;
    static uint16_t seed_salt = 0; // advances only on meaningful det knob move (>=3%)
    static float prev_seed_det_value = -1.0f; // tracks det_value at last salt increment
    if (det_gap_countdown > 0) {
      det_gap_countdown--;
    } else {

      // --- Glitch trigger probability ---
      // det_random_number (N) determines how many RPM clock cycles between glitch events.
      // It must be STABLE between ISR calls — if rand() is called every main-loop
      // iteration it can produce N=1, making counter%1==0 always true and blocking
      // the main loop with constant glitching (no RPM/AFR/gate response).
      //
      // Fix: recalculate N exactly ONCE per RPM division cycle, detected by:
      //   counter==1  (counter just reset in the ISR)
      // AND
      //   cycle_index changed  (cycle_index increments each ISR call, so this
      //                         distinguishes the one ISR call that reset the counter
      //                         from the thousands of main-loop calls while counter
      //                         is still 1 waiting for the next ISR)
      static long det_prev_cycle_index = -1;
      if (detonation_clock_cycle_reset_counter == 1 && cycle_index != det_prev_cycle_index) {
        det_prev_cycle_index = cycle_index;
        int det_trigger_range = (int)(100 * ((0.95f * ADC_MAX - current_det_value_within_type) * 0.8f) / ADC_MAX);
        if (det_trigger_range < 2) det_trigger_range = 2;
        // Minimum N=2: counter%1 is always 0, so N=1 would trigger on every counter
        // value and cause constant blocking. N>=2 keeps glitches musically spaced.
        det_random_number = (rand() % (det_trigger_range - 1)) + 2;
      }
      if (det_random_number < 2) det_random_number = 2; // safety on first entry

      // Chain-reaction guard: each counter value may only fire ONE glitch.
      // Without this, when counter % N == 0 and the ISR hasn't advanced the counter
      // yet, every main-loop call triggers a glitch back-to-back. At slow RPM the
      // ISR may not fire for 500-1000 ms, so the main loop is completely starved:
      // pot reads and TCA0 updates never happen -> pots appear frozen.
      static long det_last_glitch_counter = -1;

      if (detonation_clock_cycle_reset_counter % det_random_number == 0
          && detonation_clock_cycle_reset_counter != det_last_glitch_counter) {

        det_last_glitch_counter = detonation_clock_cycle_reset_counter;

        // --- Seed PRNG at the glitch event moment ---
        // gate/clock counters create rhythmic looping patterns tied to the gate.
        // seed_salt only advances on a >=3% det knob move, so the loop is stable
        // at a fixed position but yields new patterns after a meaningful knob change.
        if (det_value > prev_seed_det_value + DET_RESEED_THRESHOLD
            || det_value < prev_seed_det_value - DET_RESEED_THRESHOLD) {
          seed_salt++;
          prev_seed_det_value = det_value;
        }
        srand(
          ((int)((detonation_clock_cycle_reset_counter + gate_trigger_counter)
                 * ((ADC_MAX - current_det_value_within_type) * 0.8f)))
          ^ seed_salt
        );

        // --- Mute the RPM PWM output during the glitch ---
        TCA0.SINGLE.CTRLB &= ~TCA_SINGLE_CMP1EN_bm;

        // --- Glitch duration: range expands with knob (more chaos at higher positions) ---
        // At knob min: MIN to MAX_GLITCH_DURATION. At knob max: MIN to 1.5x MAX.
        uint16_t effective_max_glitch = MAX_GLITCH_DURATION + (current_det_value_within_type / 2 * MAX_GLITCH_DURATION) / ADC_MAX;
        det_glitch_duration = MIN_GLITCH_DURATION + (long)((float)(rand() / (float)RAND_MAX) * (float)(effective_max_glitch - MIN_GLITCH_DURATION + 1));

        // --- Harmonic frequency selection biased by knob position ---
        // Harmonics are integer multiples of the current RPM frequency.
        // Values {1,2,3,4,6,8,12,16} cover musically useful intervals (unison, octaves, fifths).
        const uint8_t harmonic_table[8] = {1, 2, 3, 4, 6, 8, 12, 16};
        uint8_t harm_idx_min, harm_idx_max;
        if (current_det_value_within_type < 0.4f) {
          harm_idx_min = 0;
          harm_idx_max = 3; // 1x, 2x, 3x, 4x RPM (lower, more musical)
        } else if (current_det_value_within_type < 0.6f) {
          harm_idx_min = 0;
          harm_idx_max = 4; // 1x through 8x RPM
        } else {
          harm_idx_min = 0;
          harm_idx_max = 5; // full range 1x through 16x RPM (widest bucket = most chaotic)
        }
        uint8_t selected_harmonic
          = harmonic_table[harm_idx_min + (rand() % (harm_idx_max - harm_idx_min + 1))];

        float current_rpm_freq = get_frequency_from_rpm_value(rpm_value);
        long det_frequency = (long)(current_rpm_freq * (float)selected_harmonic);
        if (det_frequency < 1L) det_frequency = 1L;
        sizeof(float);

        // --- Correct half-period calculation (Bug 2 fix) ---
        // half_period_us = 500,000 / frequency_hz
        // (The old formula used F_CPU/freq which gives CPU clock cycles, not microseconds,
        // and the uint16_t cast caused overflow for low-mid frequencies.)
        long delay_us = 500000L / det_frequency;
        if (delay_us > 65535L) delay_us = 65535L;
        if (delay_us < 1L) delay_us = 1L;

        // --- Sweep type selection: 1=Flat, 2=Ascending, 3=Descending, 4=Alternate ---
        // Type 4 (Alternate) is weighted 2x: pool of 5 gives it 40% vs 20% each for 1-3.
        static const uint8_t sweep_pool[5] = {1, 2, 3, 4, 4};
        int det_sweep = sweep_pool[rand() % 5];

        // Type 4 (Alternate): restrict starting harmonic to 2x-4x RPM regardless of bucket.
        if (det_sweep == 4) {
          uint8_t alt_harmonic = harmonic_table[(rand() % 2)]; // always {1, 2, 3}
          det_frequency = (long)(current_rpm_freq * (float)alt_harmonic);
          if (det_frequency < 1L) det_frequency = 1L;
          delay_us = 500000L / det_frequency;
          if (delay_us > 65535L) delay_us = 65535L;
          if (delay_us < 1L) delay_us = 1L;
        }

        // --- End-frequency for ascending/descending sweeps ---
        // Uses float multiply reusing __mulsf3/__fixsfsi already linked by the
        // frequency calculations above. Types 1, 4: no interpolation (flat).
        long det_freq_end = det_frequency;
        if (det_sweep == 2)
          det_freq_end = (long)(current_rpm_freq * (float)harmonic_table[harm_idx_min]);
        else if (det_sweep == 3)
          det_freq_end = (long)(current_rpm_freq * (float)harmonic_table[harm_idx_max]);
        if (det_freq_end < 1L) det_freq_end = 1L;

        // --- Alternate sweep state variables ---
        bool det_alternate_high = false;
        int det_alternate_count = 0;

        // --- Glitch duration loop ---
        // Oscillates PB0 (GLITCH_ADD) and PA7 (CORRUPT_INDICATOR_LED) at det_frequency.
        // Duration tracked in microseconds so it is independent of oscillation frequency.
        uint32_t elapsed_us = 0;
        uint32_t glitch_duration_us = (uint32_t)det_glitch_duration * 1000UL;

        while (elapsed_us < glitch_duration_us) {

          // --- Normal pulse output (one high/low half-cycle pair) ---
          hw::set_det_out(true);
          custom_delay_us((uint16_t)delay_us);
          hw::set_det_out(false);
          custom_delay_us((uint16_t)delay_us);

          elapsed_us += (uint32_t)(2UL * (uint32_t)delay_us);

          // --- Mid-loop update and early exit ---
          if (elapsed_us < glitch_duration_us) {
            //update_detonation_pot_value();
            //update_internal_rpm_pot_value();
            //update_afr_pot_value();

            if (det_value < ADC_MAX/2 || gate_trigger_counter == 1) {
              // Cancel the Glitch - knob turned down or gate reset
              break;
            }

            // --- Sweep frequency update ---
            if (det_sweep == 2 || det_sweep == 3) {
              // Linear interpolation from det_frequency → det_freq_end over
              // the full glitch duration. Both endpoints are harmonic-table
              // values so frequency is fully bounded — no runaway possible.
              // t256: left-shift avoids multiply; signed divide reuses __divmodsi4 already linked.
              // Lerp uses float multiply reusing __mulsf3/__floatsisf/__fixsfsi already linked.
              uint16_t t256 = (uint16_t)(((long)elapsed_us << 8) / (long)glitch_duration_us);
              long sweep_freq = (long)det_frequency + (long)((float)(det_freq_end - det_frequency) * ((float)t256 / 256.0f));
              if (sweep_freq < 1L) sweep_freq = 1L;
              delay_us = 500000L / sweep_freq;
              if (delay_us > 65535L) delay_us = 65535L;
              if (delay_us < 1L) delay_us = 1L;

            } else if (det_sweep == 4) {
              // Alternate sweep: octave jump up/down every 2 pulses (Bug 3 fix)
              // det_alternate_count reset is now INSIDE the trigger, not outside.
              det_alternate_count++;
              if (det_alternate_count >= 2) {
                det_alternate_count = 0; // Reset only when the jump fires
                if (det_alternate_high) {
                  delay_us = delay_us * 2; // Drop an octave (longer period)
                  if (delay_us > 65535L) delay_us = 65535L;
                } else {
                  delay_us = delay_us / 2; // Rise an octave (shorter period)
                  if (delay_us < 1L) delay_us = 1L;
                }
                det_alternate_high = !det_alternate_high;
              }
            }
            // Sweep type 1 (Flat) does not modify delay_us here.
          }
          update_detonation_pot_value();
        }

        // --- Restore oscillator with correct frequency and duty cycle after glitch ---
        // Reuse current_rpm_freq (computed at glitch start) — rpm_value is unchanged
        // since mid-glitch rpm reads were removed. Stops timer first so the direct
        // PER write is safe (same pattern as gate ISR + init_rpm_pwm). PERBUF alone
        // cannot be used here because the gate ISR resets CNT=0 faster than one RPM
        // period, preventing a natural overflow so PERBUF never transfers.
        uint32_t post_glitch_period = get_period_from_frequency(current_rpm_freq);

        TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm; // stop
        TCA0.SINGLE.PER = post_glitch_period; // new frequency
        TCA0.SINGLE.CMP1 = (uint32_t)(afr_value) * post_glitch_period / ADC_MAX; // new duty cycle
        TCA0.SINGLE.CMP1BUF = TCA0.SINGLE.CMP1; // keep buffer consistent
        TCA0.SINGLE.PERBUF = post_glitch_period; // keep buffer consistent
        TCA0.SINGLE.CNT = 0; // start fresh from BOTTOM
        TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; // restart
        TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP1EN_bm; // re-enable PWM output
        detonation_clock_mult_status = false;

        // --- Non-blocking inter-glitch gap: set countdown, return immediately ---
        // det_gap_countdown is checked at the top of the Type 6 block on every
        // process_turbo_cycle() call. While it is > 0 the glitch trigger is skipped
        // and the main loop runs freely: pot reads, TCA0 PERBUF writes, and ISRs
        // all execute normally. GAP_LOOPS_PER_MS converts ms to loop iterations.
        // As knob increases, max gap shrinks → glitches become more frequent.
        // At knob min (0.0): up to MAX_DELAY_BETWEEN_GLITCHES.
        // At knob max (0.8): up to ~20% of MAX (near MIN).
        uint16_t effective_max_gap = MIN_DELAY_BETWEEN_GLITCHES
          + (uint16_t)((1.0f - current_det_value_within_type)
                       * (float)(MAX_DELAY_BETWEEN_GLITCHES - MIN_DELAY_BETWEEN_GLITCHES));
        uint16_t gap_ms = MIN_DELAY_BETWEEN_GLITCHES
          + (uint16_t)((float)(rand() / (float)RAND_MAX)
                       * (float)(effective_max_gap - MIN_DELAY_BETWEEN_GLITCHES + 1));
        det_gap_countdown = gap_ms * GAP_LOOPS_PER_MS;
      }

    } // end else (gap not active)
  }

  hw::set_det_out(detonation_clock_mult_status && detonation_armed);
  turbo_cycle_count++;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main Loop Function
/////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(void) {
  _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, 0); // Use the 20 MHz clock

  hw::init();

  while (1) {

    process_turbo_cycle();

    // Update the period buffer with the new period.
    // Uses absolute difference — avoids float division and divide-by-zero guards.
    // Threshold 0.001 (~1 ADC LSB on 0-1 scale) keeps the same near-zero behaviour.
    uint16_t rpm_diff = abs(rpm_value - prev_rpm_value);
    uint16_t afr_diff = abs(afr_value - prev_afr_value);
    if (rpm_diff > 0.001f || afr_diff > 0.001f) {

      if (afr_value == 0) {
        //DISABLE TIMER as the period is set to 0
        TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
        // Set port A pin 7 to low
        PORTA.OUTCLR = PIN7_bm; // clear Detonate LED
      } else if (afr_value == ADC_MAX) {
        //DISABLE TIMER as the period is set to 1
        TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
      } else {
        if (!(TCA0.SINGLE.CTRLA & TCA_SINGLE_ENABLE_bm)) {
          // Start the timer back up
          TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
        }

        if (afr_diff > 0.001f) {
          TCA0.SINGLE.CMP1BUF = (uint32_t)(afr_value) * TCA0.SINGLE.PER / ADC_MAX;
        }

        if (rpm_diff > 0.001f) {
          // Do necessary calculations for RPM changes
          float frequency = get_frequency_from_rpm_value(rpm_value);
          uint32_t period = get_period_from_frequency(frequency);

          // Capture the current counter value and current period
          uint16_t current_count = TCA0.SINGLE.CNT;
          uint16_t current_period = TCA0.SINGLE.PER;

          // Calculate the proportion of the current cycle that has elapsed
          // Calculate the new counter value to maintain the phase
          /* TODO: unused?
          uint16_t new_count = period * current_count / current_period;

          // Ensure the counter value is within the valid range
          if (new_count >= period) {
            new_count = period - 1; // Reset to 0 if it exceeds the period
          }
          */

          TCA0.SINGLE.PERBUF = period;

          // Synchronize CMP1 to the new period so the compare always fires.
          // Without this, CMP1 > PER after a rapid low->high sweep, and the
          // output gets stuck HIGH because the compare match never triggers.
          uint32_t new_cmp1 = (uint32_t)(afr_value) * period / ADC_MAX;
          if (new_cmp1 >= period) new_cmp1 = period - 1;
          TCA0.SINGLE.CMP1BUF = new_cmp1;
        }
      }

      // Store previous values for comparison
      prev_rpm_value = rpm_value;
      prev_afr_value = afr_value;
    }
  }
}
