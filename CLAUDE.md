# CLAUDE.md — Space Ships

## Project
- Two model spaceships shuttle on rope-and-pulley lines. Each ship has one 12V DC motor, reversed by 2 relays and stopped by a limit switch at each end.
- Each ship's cycle: move → hit limit → stop → 5 s dwell → reverse, repeating while powered. The two ships run independently.
- Deployment: unattended public display, runs continuously while powered.
- Firmware safety layers: move timeout, wrong-way check, both-limits check, 3-fault lockout, ~2 s watchdog.

## Hardware
- Board: Arduino Nano Every (ATmega4809), 5V logic, powered from 12V on VIN. The VIN regulator accepts up to 21V (ABX00028 datasheet, p.2). A0–A7 work as digital GPIO (ABX00028, §6.2) and map to D14–D21 (core `pins_arduino.h`, nona4809).
- D14 (A0) → Ship 1 relay, to POS1
- D16 (A2) → Ship 1 relay, to POS2
- D18 (A4) → Ship 2 relay, to POS1. Also I2C SDA: don't start Wire while it drives a relay.
- D20 (A6) → Ship 2 relay, to POS2
- D6 ← Ship 1 limit, POS1
- D7 ← Ship 1 limit, POS2
- D8 ← Ship 2 limit, POS1
- D9 ← Ship 2 limit, POS2
- D13 → onboard status LED: slow blink = OK, fast blink = fault
- D0/D1 reserved (hardware USART). Spare: D2–D5, D10–D12, D15 (A1), D17 (A3), D19 (A5), D21 (A7).
- Relays: 12V optocoupler modules, the same type as the Castle Scenes project. Trigger polarity to be verified on the bench, then set `RELAY_ON`.
- Limit switches: assumed NO to GND with pull-up (`LIMIT_ACTIVE LOW`). Unverified until the bench test.
- Limit input filter: 10 kΩ pull-up, 1 kΩ series, 100 nF to GND at the pin. OK ranges: pull-up 2.2–10 kΩ, series 470 Ω–2.2 kΩ (keep small vs the 20–50 kΩ internal pull-up), cap 47–470 nF (ATmega4809 DS40002016B, Table 4-16).
- 12V supply: unknown. Motor running and stall current: unknown. No input fuse.
- Buses: none. Serial: USB at 115200 baud.

## Toolchain and commands
- VS Code + PlatformIO on Ubuntu Linux; repo on GitHub.
- Envs: `mw_space_two_simple_ships` is the live display firmware; `bench_test` is a serial menu to click each relay, read the limit switches, time ship travel for tuning, and jog ships by holding 5–8.

## Code standards
- Config constants at the top of each file, so tuning never means hunting through logic.
- `F()` on every string literal, to save SRAM.
- Check every sensor read and bus transaction and report failures: fail loudly, not silently.
- Small single-purpose functions, because they're easier to test and debug.
- Comments explain why, not what. One short line where possible.
- Pin every library version in `lib_deps` for reproducible builds. None are used yet.
- Debug output behind `#define DEBUG 1/0`, one switch to silence it for deployment.
- No `delay()` in the live loop, because it blocks the watchdog and the other ship.
- Motor control goes only through `motorStop/ToPos1/ToPos2`, and polarity is set only in the `RELAY_ON` and `LIMIT_ACTIVE` defines. Why: each rule is enforced in one place.

## Workflow rules
- Direct, literal language, in bullets. Minimal explanation unless I ask; give the why in one short clause.
- Plain text. Standard markdown tables are OK (I document in Notion now, Obsidian later).
- Work in steps of 15–30 minutes. End each reply with done / next.
- Run `pio run` after every change. Fix errors and warnings before reporting done.
- Never upload, or run anything that moves hardware, without asking first. The firmware drives motors.
- Commit and push only when I ask.
- Never put secrets, credentials, personal details, or client names in tracked files, because the repo is public.

## Verification rules
- For specs (voltage and current limits, pinouts, timing, library behavior), cite the datasheet or official docs URL.
- If no source covers it, mark it "unverified". Don't extrapolate.
- Don't invent hardware facts. Ask, or mark them unknown.
- Bench values (relay polarity, NO/NC, travel times) come from me. Change constants only after I report them.
- After each build, report flash and RAM use. The 4809 has 48 KB flash and 6 KB SRAM (ABX00028, p.2).

## Sources
- Nano Every datasheet: https://docs.arduino.cc/resources/datasheets/ABX00028-datasheet.pdf
- PlatformIO, Nano Every board: https://docs.platformio.org/en/latest/boards/atmelmegaavr/nano_every.html
