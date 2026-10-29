# Space Ships — Bill of Materials

As of 2026-10-24. Redesign v0.4: relays replaced by one DRV8871 motor driver per ship.

Parts for one complete Space Ships controller (2 ships, 2 motors, 2 drivers, 4 limit switches). Matches the v0.4 pin map in `CLAUDE.md` and `ref/wiring-diagram.svg` (v0.4).

## Controller board

All through-hole, on perfboard. The Nano and both drivers sit in female headers so each lifts out for replacement.

| Ref | Qty | Part | Value / rating | Why |
| --- | --- | --- | --- | --- |
| U1 | 1 | Arduino Nano Every (ABX00028) | 5V logic, VIN 12V | Controller |
| — | 2 | Female header, 1×15 | 2.54 mm pitch | Socket for U1 |
| U2–U3 | 2 + 2 spare | Adafruit DRV8871 motor driver (product 3190) | 6.5–45 V, 3.6 A peak; current limit about 2.1 A with the fitted 30 kΩ R_ILIM | One per ship: direction, PWM speed, current limit, no contacts to wear |
| — | to suit | Male header pins (on U2–U3) + female headers (on board) | 2.54 mm | Plug-in drivers. Check the driver's power and output pad pitch fits 0.1" headers |
| C1 | 1 | Electrolytic capacitor | 1000 µF, 25 V min | 12V bulk at the board input |
| C2 | 1 | Electrolytic capacitor | 10 µF, 25 V min | Local reservoir at Nano VIN |
| C3 | 1 | Ceramic capacitor | 100 nF, 50 V, X7R | Decoupling at Nano VIN, close to the pin |
| C8–C9 | 2 | Electrolytic capacitor | 47 µF, 25 V min | Bulk at each driver VM (TI typical application) |
| C10–C11 | 2 | Ceramic capacitor | 100 nF, 50 V | Bypass at each driver VM, close to the pins (TI typical application) |
| R1–R4 | 4 | Resistor | 10 kΩ, ¼ W (2.2–10 kΩ OK) | Limit input pull-ups to +5V |
| R5–R8 | 4 | Resistor | 1 kΩ, ¼ W | Limit input series resistor; RC filter with C4–C7 |
| C4–C7 | 4 | Ceramic capacitor | 100 nF, 50 V | Limit input filter, at the Arduino pin |
| F1 | 1 + spares | Fuse, 5×20 mm, time-lag (slow-blow), ceramic | 3.15 A (or 3 A) | 12V input protection. Time-lag rides through C1 charging and motor starts |
| FH1 | 1 | PCB fuse holder BLX-A, 5×20 mm | 250 V, 6.3 A | Tool-free fuse swap. Test-fit on the perfboard before soldering |
| J1 | 1 | Screw terminal, 2-pos | 5.08 mm | 12V input |
| J2–J5 | 4 | Screw terminal, 2-pos | 3.5 or 5.08 mm | Limit switches, signal + GND |
| J6–J7 | 2 | Screw terminal, 2-pos | 3.5 or 5.08 mm | Motor outputs (from each driver OUT1/OUT2) |
| — | 1 | Perfboard | ~70 × 90 mm | Board |
| — | 4 | M3 standoffs + screws | — | Mount board in enclosure |
| LED1, R13 | 1 each | 5 mm LED + resistor (optional) | 1 kΩ | Panel status light on D13; only if wanted |

No external pull-downs on the driver inputs: the DRV8871 has 100 kΩ internal pull-downs on IN1/IN2, so a Nano reset or power loss leaves the driver in coast/sleep.

## Off-board parts

| Ref | Qty | Part | Value / rating | Why |
| --- | --- | --- | --- | --- |
| SW1–SW4 | 4 | Limit microswitch | NC wired (bench-verified 2026-10-19) | End-of-travel |
| — | as needed | Twisted-pair cable | 22 AWG | Limit switch runs; keeps noise pickup low |
| — | as needed | Hook-up wire, red + black | 22 AWG | 12V power and motor runs |
| — | 1 | Enclosure | Size TBD | Location and access |
| M1–M2, PSU | — | Motors and 12V supply | Existing (client) | Motors run under 1 A; supply specs pending |

## Removed in v0.4

| Part | Why |
| --- | --- |
| K1–K4 relay modules, R9–R12 pull-downs, relay headers, Dupont jumpers | Replaced by U2–U3 |
| D1–D2 TVS diodes, motor-side 100 nF caps (never fitted) | Not needed with the driver; a cap across a PWM-driven motor causes current spikes |

## Pending values

| Item | Waiting on | How it's decided |
| --- | --- | --- |
| Ship speeds and STEADY/RAMP settings | Speed-setting discussion | Per ship |
| Travel times, MOVE_TIMEOUT | On-site timing after install | 1.5× slowest average, re-timed at final speeds |
| PWM frequency | Firmware testing | Default ~1 kHz may whine; raising it interacts with Nano timing |
| Driver pad pitch vs 0.1" headers | Board in hand | Check before soldering headers |
| LED1 / R13 | Client preference | Fit only if staff want a visible status light |

## Sources

- [Arduino Nano Every datasheet (ABX00028)](https://docs.arduino.cc/resources/datasheets/ABX00028-datasheet.pdf) — VIN up to 21 V, pinout, PWM pins
- [TI DRV8871 datasheet](https://www.ti.com/lit/gpn/drv8871) — 6.5–45 V, 3.6 A peak, I_TRIP = 64 / R_ILIM (kΩ), 100 kΩ input pull-downs, 0.1 µF + 47 µF typical VM capacitors
- [Adafruit DRV8871 breakout guide (product 3190)](https://learn.adafruit.com/adafruit-drv8871-brushed-dc-motor-driver-breakout/overview) — 30 kΩ R_ILIM fitted (~2 A)
- [Littelfuse 215 series 5×20 mm time-lag fuse datasheet](https://littelfuse.com/%7E/media/electronics/datasheets/fuses/littelfuse_fuse_215_datasheet.pdf.pdf) — time-lag characteristics (example part)
