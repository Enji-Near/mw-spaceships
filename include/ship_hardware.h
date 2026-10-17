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

// Pin level when a ship IS at a limit.
// NO switch to GND (current assumption): LOW.   NC switch to GND (preferred): HIGH.
// VERIFY on the bench with a multimeter in continuity mode.
#define LIMIT_ACTIVE  LOW

/****** Pin Configuration ******/
// Relays (OUTPUT)
const uint8_t SHIP1_RELAY_TO_POS1 = 2;
const uint8_t SHIP1_RELAY_TO_POS2 = 3;
const uint8_t SHIP2_RELAY_TO_POS1 = 4;
const uint8_t SHIP2_RELAY_TO_POS2 = 5;

// Limit switches (INPUT_PULLUP)
const uint8_t SHIP1_LIMIT_POS1 = 6;
const uint8_t SHIP1_LIMIT_POS2 = 7;
const uint8_t SHIP2_LIMIT_POS1 = 8;
const uint8_t SHIP2_LIMIT_POS2 = 9;

// Status
const uint8_t STATUS_LED = LED_BUILTIN;  // D13

// Watchdog constant differs between chip families.
#ifdef ARDUINO_ARCH_MEGAAVR
  #define WDT_TIMEOUT  WDT_PERIOD_2KCLK_gc  // ATmega4809: ~2 s
#else
  #define WDT_TIMEOUT  WDTO_2S              // Classic AVR (Uno/Nano)
#endif
