#pragma once
#include <Arduino.h>

/*****  Space Ships — shared hardware config  *****
 *  Used by both envs (live + bench_test), so polarity and pins are set in one place.
 *  Bench-verified values go here; both firmwares pick them up on the next build.
 **************************************************/

/****** Hardware Options — set to match the parts on hand ******/
// Relay module input polarity. Modules with the trigger jumper set to H: HIGH = ON.
// VERIFY on the bench: write RELAY_ON, listen for the click; write RELAY_OFF, confirm release.
#define RELAY_ON    HIGH
#define RELAY_OFF   LOW

// Pin level when a ship IS at a limit, per switch.  NO to GND: LOW.  NC to GND: HIGH.
// Bench-verified 2026-10-19 (12V travel test): all NC.
#define SHIP1_POS1_ACTIVE  HIGH  // NC
#define SHIP1_POS2_ACTIVE  HIGH  // NC
#define SHIP2_POS1_ACTIVE  HIGH  // NC
#define SHIP2_POS2_ACTIVE  HIGH  // NC

/****** Pin Configuration ******/
// Relays (OUTPUT) — analog header pins used as digital, every second pin for board layout.
// A4 (D18) is also I2C SDA: don't start Wire while it drives a relay.
// "TO_POS1" relays: COM wired to motor + (bench report). Actual travel direction unverified until 12V test;
// if a ship runs the wrong way, swap that ship's two pin assignments here.
// Ship 1 swapped 2026-10-19: A0 relay drove the ship toward POS2. Verified by 12V timing run.
const uint8_t SHIP1_RELAY_TO_POS1 = A2;  // D16
const uint8_t SHIP1_RELAY_TO_POS2 = A0;  // D14
const uint8_t SHIP2_RELAY_TO_POS1 = A4;  // D18
const uint8_t SHIP2_RELAY_TO_POS2 = A6;  // D20

// Limit switches (INPUT_PULLUP). As built: POS2 on the lower pin of each pair (bench report 2026-10-19).
const uint8_t SHIP1_LIMIT_POS1 = 7;
const uint8_t SHIP1_LIMIT_POS2 = 6;
const uint8_t SHIP2_LIMIT_POS1 = 9;
const uint8_t SHIP2_LIMIT_POS2 = 8;

// Status
const uint8_t STATUS_LED = LED_BUILTIN;  // D13

// Watchdog constant differs between chip families.
#ifdef ARDUINO_ARCH_MEGAAVR
  #define WDT_TIMEOUT  WDT_PERIOD_2KCLK_gc  // ATmega4809: ~2 s
#else
  #define WDT_TIMEOUT  WDTO_2S              // Classic AVR (Uno/Nano)
#endif
