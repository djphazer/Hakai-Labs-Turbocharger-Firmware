# Hakai Labs Turbocharger - Firmware

The official open-source firmware for the **Hakai Labs Turbocharger**, a Eurorack
module built on the Microchip ATtiny804.

At its core, Turbocharger is a simple super high-speed switch (not a VCA) between
two inputs. That sounds boring, but the switch is run by a CMOS switch IC capable
of switching at extremely high speeds. Paired with the internal oscillator and its
PWM controls, that makes for some fun experimentation. And you don't even have to
use the internal oscillator - there's an external clock input, so you can patch in
your own pulse wave to drive the switch.

This firmware is what runs on the module's ATtiny804: it generates the internal
"engine RPM" oscillator that toggles the switch, gives you PWM control over the
pulse, and adds the **detonation** glitch engine - random, harmonically-related
bursts layered on top. This repo is open so you can read how it works, modify it,
and flash your own firmware onto the module.

## Controls & I/O

The main signal path (Input A/B -> CMOS switch -> Wastegate -> Main Out) is all
analog. The firmware's job is to read the three control knobs/CVs plus the gate
and external clock, then generate the RPM clock and detonation glitches that
actually drive the switch.

**Knobs** - each is summed with its matching CV input, acting as an offset (not an
attenuator):

- **RPM** - speed of the internal oscillator that runs the main switch (roughly
  1.5 Hz to 1.5 kHz on an exponential curve).
- **AFR** - pulse width of the RPM oscillator (50% between Input A and B at noon).
- **Detonation** - the chaos control. Fully left disarms glitches; left of center
  injects loose clock divisions; past center throws in random noise and
  oscillations that get denser as you turn up.

**CV inputs** (-5 V to +5 V, offset by the matching knob): **RPM CV**,
**AFR/PWM CV**, **Detonation CV**.

**Signal & clock I/O**

- **Input A / Input B** - the two main signal inputs (-12 V to +12 V) routed
  through the CMOS switch. Normalled to 5 V (A) and 0 V (B) when unpatched.
- **Input A Thru / Input B Thru** - buffered pass-throughs of the two inputs.
- **Main Out** - the switched output.
- **Gate (Wastegate) In** - 0-5 V. Low forces the Main Out to 0 V; also loops the
  detonation glitches rhythmically. Normalled to 5 V (output open) when unpatched.
- **Ext. Clock In (External RPM)** - 0-5 V. Overrides the internal oscillator so
  you can clock the switch from your own pulse source.

## Hardware

Everything you need to write your own firmware - chip specs, full pinout,
peripherals, and build/flash instructions - is in **[HARDWARE.md](HARDWARE.md)**.

Quick summary: **ATtiny804** (8-bit AVR, 20 MHz, 8 KB flash, 512 B SRAM), 14-pin
SOIC, programmed over UPDI.

## Building & flashing

The firmware is written against bare AVR registers (no Arduino HAL), so it builds
under **[megaTinyCore](https://github.com/SpenceKonde/megaTinyCore)**:

1. Install megaTinyCore via the Arduino Boards Manager.
2. Select board **ATtiny804**, **Clock: 20 MHz internal**.
3. There's no bootloader - flash over **UPDI** with a SerialUPDI adapter,
   jtag2updi, or an Atmel-ICE/SNAP, then use **Sketch -> Upload Using Programmer**.

See [HARDWARE.md](HARDWARE.md) for wiring and fuse details.

## Repository contents

- `Hakai-Labs-Turbocharger-Firmware.ino` - the firmware.
- `HARDWARE.md` - chip, pinout, and flashing reference.
- `README.md` - this file.

## Contributing

Modifications and experiments are welcome. Feel free to open an issue or pull
request with improvements, or fork it and make it your own. The firmware header
lists a few tuning "todos" (glitch frequency and density) if you're looking for a
place to start.

## License

Released under the [MIT License](LICENSE) - you're free to use, modify, and
redistribute it, including in commercial projects, with attribution.

---

*Hakai Labs Turbocharger · ATtiny804 · Firmware v3.1*
