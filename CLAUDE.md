# CLAUDE.md — Space Ships

## Project
- Two model spaceships shuttle on rope-and-pulley lines. Each ship has one 12V DC motor, driven by its own DRV8871 H-bridge (direction + PWM speed) and stopped by a limit switch at each end.
- Each ship's cycle: move → hit limit → stop → 5 s dwell → reverse, repeating while powered. The two ships run independently.
- Deployment: unattended public display, runs continuously while powered.
- Firmware safety layers: move timeout, wrong-way check, both-limits check, 3-fault lockout, ~2 s watchdog.

## Hardware
- Board: Arduino Nano Every (ATmega4809), 5V logic, powered from 12V on VIN. The VIN regulator accepts up to 21V (ABX00028 datasheet, p.2). A0–A7 work as digital GPIO (ABX00028, §6.2) and map to D14–D21 (core `pins_arduino.h`, nona4809).
- Redesign v0.4 (2026-10-24): relays replaced by one Adafruit DRV8871 board (product 3190) per ship. Firmware v0.4 written 2026-10-24, not yet bench-tested.
- D9 → Ship 1 driver IN1 (PWM)
- D10 → Ship 1 driver IN2 (PWM)
- D3 → Ship 2 driver IN1 (PWM)
- D5 → Ship 2 driver IN2 (PWM)
- Which input drives toward POS1 is set on the bench; if a ship runs the wrong way, swap that ship's IN1/IN2 assignment.
- D7 ← Ship 1 limit, POS1
- D6 ← Ship 1 limit, POS2
- D4 ← Ship 2 limit, POS1 (moved from D9 in v0.4)
- D8 ← Ship 2 limit, POS2
- D13 → onboard status LED: slow blink = OK, fast blink = fault
- D0/D1 reserved (hardware USART). Spare: D2, D11, D12, A0–A7 (D14–D21).
- PWM pins on the Nano Every: D3, D5, D6, D9, D10 (core variant timer map). They share a clock with millis() (core `variant.c`, `wiring.c`), so changing PWM frequency needs care. Unverified until tested.
- Driver logic (DRV8871 datasheet §7.3.1): IN1/IN2 = 0/0 coast then sleep after ~1 ms; 1/0 forward; 0/1 reverse; 1/1 brake. Inputs have 100 kΩ internal pull-downs (§6.5).
- Stop = brake (1/1) at the end of each trip, during dwell, and in fault/lockout. Nano reset or power loss = inputs low = coast (motor unpowered; ship may drift).
- Driver current limit: I_TRIP = 64 / R_ILIM (kΩ) (§7.3.3); Adafruit fits 30 kΩ ≈ 2.1 A. Overcurrent shutdown 3.7–6.4 A with ~3 ms retry (§7.3.5.2).
- Driver VM: 100 nF ceramic + 47 µF / 25 V electrolytic per driver (datasheet typical application).
- Limit switches (bench-verified 2026-10-19): all NC to GND (active HIGH), with pull-up.
- Limit input filter: 10 kΩ pull-up, 1 kΩ series, 100 nF to GND at the pin. OK ranges: pull-up 2.2–10 kΩ, series 470 Ω–2.2 kΩ (keep small vs the 20–50 kΩ internal pull-up), cap 47–470 nF (ATmega4809 DS40002016B, Table 4-16).
- Fuse: F1, 5×20 mm time-lag 3.15 A at the 12V input, in a BLX-A PCB holder. The drivers only protect their own outputs.
- Wiring: 22 AWG throughout, including the ~15' 12V run (about 1V drop at 2A; slower ships are acceptable).
- 12V supply: unknown. Motors run under 1 A; stall current unknown.
- Buses: none. Serial: 115200 baud (bench test and debug).

## Toolchain and commands
- VS Code + PlatformIO on Ubuntu Linux; repo on GitHub.
- Envs: `mw_space_two_simple_ships` is the live display firmware; `bench_test` is a serial menu to jog ships (hold 5–8), watch the limit switches (w), calibrate steady speed (t), minimum speed (m) and ramp (r), and print the calibration block for `include/ship_config.h` (p).
- Shared code: `include/ship_motion.h` + `src/ship_motion.cpp`, built into both envs.

## Code standards
- Every value that is or could be tuned (pins, polarity, timings, speeds, calibration results) lives in one file: `include/ship_config.h`. Why: one place to tune, nothing to hunt for.
- `F()` on every string literal, to save SRAM.
- Check every sensor read and bus transaction and report failures: fail loudly, not silently.
- Small single-purpose functions, because they're easier to test and debug.
- Comments explain why, not what. One short line where possible.
- Pin every library version in `lib_deps` for reproducible builds. None are used yet.
- Debug output behind `#define DEBUG 1/0`, one switch to silence it for deployment.
- No `delay()` in the live loop, because it blocks the watchdog and the other ship.
- Motor control goes only through the motor helper functions. Code used by both firmwares (motion, limits, motor control) lives in one shared file. Why: what is tuned on the bench behaves the same live; standard for all projects.

## Workflow rules
- Direct, literal language, in bullets. Minimal explanation unless I ask; give the why in one short clause.
- Plain text. Standard markdown tables are OK (I document in Notion now, Obsidian later).
- Work in steps of 15–30 minutes. End each reply with done / next.
- Run `pio run` after every change. Fix errors and warnings before reporting done.
- Never upload, or run anything that moves hardware, without asking first. The firmware drives motors.
- Commit and push only when I ask.
- Keep it simple: smallest solution that works. No adaptive or learning behavior; values are set once and kept.
- Don't add behavior I didn't ask for. If an extra safety check seems needed, propose it and wait for my answer.
- Never put secrets, credentials, personal details, or client names in tracked files, because the repo is public.

## Verification rules
- For specs (voltage and current limits, pinouts, timing, library behavior), cite the datasheet or official docs URL.
- If no source covers it, mark it "unverified". Don't extrapolate.
- Don't invent hardware facts. Ask, or mark them unknown.
- Bench values (direction mapping, NO/NC, travel times) come from me. Change constants only after I report them.
- After each build, report flash and RAM use. The 4809 has 48 KB flash and 6 KB SRAM (ABX00028, p.2).

## Sources
- Nano Every datasheet: https://docs.arduino.cc/resources/datasheets/ABX00028-datasheet.pdf
- PlatformIO, Nano Every board: https://docs.platformio.org/en/latest/boards/atmelmegaavr/nano_every.html
