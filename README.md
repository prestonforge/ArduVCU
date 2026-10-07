# ArduVCU — low-cost racing telemetry for Greenpower F24

**A £55 open-source vehicle control unit that gives Greenpower F24 drivers live battery telemetry and predictive pacing — built because commercial systems cost £100+ and junior teams drive blind.**

Awarded the **Gold CREST Award** by the British Science Association (April 2026) — 78 logged project hours.

## The problem

Greenpower F24 teams race on a fixed 24V lead-acid battery. The driver has no instrumentation: they can't see how fast the battery is actually draining, so they can't pace. The result is the "Feedback Gap" — batteries die before the finish line, or the car crawls home with charge to spare. Commercial telemetry (e.g. eChook Nano) costs £100+ and needs a smartphone, which puts it out of reach for most school teams.

## How it works

An ESP32 reads the traction battery through isolated sensing and turns raw current/voltage into **live minutes-remaining**, using Peukert's Law — because lead-acid batteries deliver *less* capacity the harder you pull current:

- **Predictive runtime** — Peukert estimation implemented from scratch in C++, no libraries
- **O(1) signal filtering** — a circular-buffer moving average (10 samples, modulo wrap-around) that tames motor EMI without re-summing on every loop
- **Hysteresis safety state machine** — "Limp Mode" at 21.0 V deep sag; full power only returns once voltage recovers by a 1.5 V dead-band. No relay chatter, no welded contactors
- **Isolated architecture** — 24 V traction power stepped down via a 100k/10k divider; current measured by a galvanically isolated ACS712 Hall-effect sensor (up to 30 A)
- **12-bit ADC** — 4,096 steps vs the 1,024 of typical 10-bit microcontrollers, enough to catch micro-fluctuations in current draw
- **Digital twin — real and open-source** — the 0.05 V/min pacing budget comes from a first-order lead-acid discharge model (Peukert's Law + internal-resistance sag). Two working, auditable versions ship with this repo: an interactive browser twin → [prestonforge.github.io/digital-twin.html](https://prestonforge.github.io/digital-twin.html), and a full Excel model with live formulas in `/docs`
- **Fusion 360 enclosure** — ABS (not PLA: it warps at 60 °C next to a hot motor), brass threaded inserts, U-shaped cable slots for track vibration

## A documented pivot

The original design drove an I2C OLED cockpit display. It never initialised. After a systematic debug (I2C scanner, physical-layer checks, address verification) the project pivoted to **headless serial telemetry** — streaming at 2 Hz to a ground-station laptop. Better outcome: full data for aerodynamic tuning instead of a 0.96-inch driver display.

## Bill of materials (≈£55)
   Item | Purpose |
 |---|---|
 | ESP32 DevKit (12-bit ADC) | Core MCU + telemetry |
 | ACS712 Hall-effect sensor (30 A) | Isolated current measurement |
 | 100k/10k resistor divider | 24 V → logic-level voltage sense |
 | Relay + contactor interface | Motor safety cut-off |
 | Enclosure (ABS, FDM-printed) | Vibration-resistant housing |

Commercial equivalent: ~£100+ plus an Android phone.

## Status

- [x] Breadboard prototype — working, logged 78 hrs, Gold CREST assessed
- [ ] Custom PCB port (from breadboard)
- [ ] Race-day installation + live pacing validation
- [ ] Full open-source toolchain release for school teams

## License

MIT — see . Built for school STEM racing teams; use it, improve it, pass it on.
