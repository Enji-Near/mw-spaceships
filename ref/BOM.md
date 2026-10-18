# Space Ships — Bill of Materials

As of 2026-10-16.

Parts for one complete Space Ships controller (2 ships, 2 motors, 4 relays, 4 limit switches). Matches `include/ship_hardware.h` and `ref/wiring-diagram.svg` (v0.2).

## Controller board

All through-hole, on perfboard. The Nano sits in female headers so it lifts out for replacement.

| Ref | Qty | Part | Value / rating | Why |
| --- | --- | --- | --- | --- |
| U1 | 1 | Arduino Nano Every (ABX00028) | 5V logic, VIN 12V | Controller |
| — | 2 | Female header, 1×15 | 2.54 mm pitch | Socket for U1, swap without desoldering |
| C1 | 1 | Electrolytic capacitor | 1000 µF, 25 V min (470 µF OK) | 12V bulk: covers motor start-up dips |
| C2 | 1 | Electrolytic capacitor | 10 µF, 25 V min | Local reservoir at Nano VIN |
| C3 | 1 | Ceramic capacitor | 100 nF, 50 V, X7R | High-frequency decoupling at Nano VIN, close to the pin |
| R1–R4 | 4 | Resistor | 10 kΩ, ¼ W (2.2–10 kΩ OK) | Limit input pull-ups to +5V; stiffer than the internal pull-up |
| R5–R8 | 4 | Resistor | 1 kΩ, ¼ W | Limit input series resistor; forms RC filter with C4–C7, limits surge current |
| C4–C7 | 4 | Ceramic capacitor | 100 nF, 50 V | Limit input filter, at the Arduino pin |
| R9–R12 | 4 | Resistor | 10 kΩ, ¼ W | Pull-down on each relay IN line; holds relays off while the Nano resets or uploads |
| J1 | 1 | Screw terminal, 2-pos | 5.08 mm, ≥ 10 A | 12V input |
| J2–J5 | 4 | Screw terminal, 2-pos (pluggable preferred) | 3.5 or 5.08 mm | Limit switches, signal + GND |
| J6–J9 | 4 | Male header, 1×3 | 2.54 mm | To relay modules: 12V, GND, IN |
| — | 1 | Perfboard | ~70 × 90 mm | Board |
| — | 4 | M3 standoffs + screws | — | Mount board in enclosure |
| LED1, R13 | 1 each | 5 mm LED + resistor (optional) | 1 kΩ | Panel status light on D13; only if client wants it (question O5) |

## Off-board parts

These mount in the enclosure or at the motors, not on the perfboard. Relay modules have their own screw terminals, so motor wires land directly on them.

| Ref | Qty | Part | Value / rating | Why |
| --- | --- | --- | --- | --- |
| K1–K4 | 4 + 2 spare | 12V 1-channel relay module, optocoupler, H/L trigger jumper | 12V coil; contacts 10 A at 30 VDC | Motor direction; set jumper to H. Pending client relay inventory |
| D1–D2 | 2 | TVS diode, bidirectional, P6KE18CA | 15.3 V standoff, 600 W, DO-15 | Across each motor's terminals; clamps spikes that arc relay contacts |
| C8–C9 | 2 | Ceramic capacitor | 100 nF, 50 V | Across each motor's terminals, soldered at the motor; cuts brush noise |
| F1 | 1 | Inline fuse holder + slow-blow fuse | Rating TBD | Not fitted: no input fuse (decided 2026-10-16) |
| SW1–SW4 | 4 | Limit microswitch | C/NO/NC terminals preferred | End-of-travel; check what the client has (question M3–M4) |
| — | 12 | Jumper wire, female–female | 2.54 mm Dupont | Board J6–J9 to relay module inputs (3 per module) |
| — | as needed | Twisted-pair cable | 22 AWG | Limit switch runs; keeps noise pickup low |
| — | as needed | Hook-up wire, red + black | 22 AWG (decided 2026-10-16; slower ships acceptable) | 12V power and motor runs |
| — | 1 | Enclosure | Size TBD | Client question I1 |
| M1–M2, PSU | — | Motors and 12V supply | Existing (client) | Not purchased; specs pending |

## Alternate: client's RZ-12-C relays

Use this only if the client prefers their RZ-12-C stock over relay modules. Its 1 A contacts may be too small for the motors, so measure motor current first.

| Change | Qty | Part | Value / rating | Why |
| --- | --- | --- | --- | --- |
| Replaces K1–K4 modules | 4 | RZ-12-C relay | 12V coil, DPDT, 1 A contacts | Client stock |
| Add | 4 | Relay socket | To match RZ-12-C pinout; check one in hand | Swap without desoldering |
| Add U2 | 1 | ULN2003A + DIP-16 socket | 500 mA per channel | Drives 12V coils from 5V pins; has built-in flyback diodes |
| Remove | — | R9–R12 (10 kΩ) | — | ULN2003A has internal input pull-downs |
| Remove | — | J6–J9, jumper wires | — | Relays sit on the board |

Code stays the same (RELAY_ON = HIGH) in both options.

## Pending values

The board can be built now. The items below land off-board and can be added once measured or confirmed.

| Item | Waiting on | How it's decided |
| --- | --- | --- |
| F1 fuse rating | Motor running current | Closed: no input fuse fitted (decided 2026-10-16) |
| Motor wire gauge | Motor running current | Closed: 22 AWG throughout, about 1 V drop on a 15' run at 2 A; slower ships acceptable |
| Relay part (modules vs RZ-12-C) | Client relay inventory (R1–R3); motor stall current | Stall current ≈ 12 V ÷ winding resistance; must sit within contact rating |
| Limit switch wiring (NO vs NC) | Bench test of the switches | NC needs LIMIT_ACTIVE = HIGH in the sketch |
| LED1 / R13 | Client question O5 | Fit only if staff want a visible status light |
| Enclosure | Client question I1 | Location and access |

## Sources

- [Arduino Nano Every datasheet (ABX00028)](https://docs.arduino.cc/resources/datasheets/ABX00028-datasheet.pdf) — VIN regulator up to 21 V, pinout
- [P6KE18CA, element14 listing](https://nz.element14.com/yageo/p6ke18ca-b/tvs-diode-bidir-15-3v-600w-do/dp/3788243) — bidirectional, 15.3 V standoff, 600 W, DO-15
- [12V 1-channel relay module user guide (Phipps Electronics)](https://www.phippselectronics.com/support/users-guide-12v-1-channel-relay-module-high-low-trigger-optocoupler/) — 3.3/5 V trigger, H/L jumper, 10 A at 30 VDC
- [TI ULN2003A datasheet](https://www.ti.com/lit/ds/symlink/uln2003a.pdf) — 5 V input, 500 mA outputs, clamp diodes, internal pull-downs
- [NTE R40 series datasheet (RZ-12-C equivalent)](https://cdn.eicom.ru/media/PDF/4310510.pdf) — coil and contact ratings
