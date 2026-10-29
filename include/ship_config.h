#pragma once
#include <Arduino.h>

/*****  Space Ships — ALL tunable values  *****
 *  Used by both envs (live + bench_test). Nothing tunable lives anywhere else.
 *  Bench values (direction, NO/NC, speeds) come from bench tests, then get set here.
 **********************************************/

/****** Debug ******/
#define DEBUG  1  // Live firmware serial output: 1 = on (115200 baud), 0 = deployment

/****** Driver pins (DRV8871 IN1/IN2, all PWM-capable) ******/
// Direction unverified until bench test: if a ship runs the wrong way, swap that ship's two values.
const uint8_t SHIP1_IN_TO_POS1 = 9;   // Ship 1 IN1
const uint8_t SHIP1_IN_TO_POS2 = 10;  // Ship 1 IN2
const uint8_t SHIP2_IN_TO_POS1 = 3;   // Ship 2 IN1
const uint8_t SHIP2_IN_TO_POS2 = 5;   // Ship 2 IN2

/****** Limit switches (INPUT_PULLUP) ******/
const uint8_t SHIP1_LIMIT_POS1 = 7;
const uint8_t SHIP1_LIMIT_POS2 = 6;
const uint8_t SHIP2_LIMIT_POS1 = 4;   // Moved from D9 in v0.4
const uint8_t SHIP2_LIMIT_POS2 = 8;

// Pin level when a ship IS at a limit. NO to GND: LOW. NC to GND: HIGH. Bench-verified 2026-10-19: all NC.
#define SHIP1_POS1_ACTIVE  HIGH
#define SHIP1_POS2_ACTIVE  HIGH
#define SHIP2_POS1_ACTIVE  HIGH
#define SHIP2_POS2_ACTIVE  HIGH

/****** Status ******/
const uint8_t STATUS_LED = LED_BUILTIN;  // D13: slow blink = OK, fast blink = fault

#ifdef ARDUINO_ARCH_MEGAAVR
  #define WDT_TIMEOUT  WDT_PERIOD_2KCLK_gc  // ATmega4809: ~2 s
#else
  #define WDT_TIMEOUT  WDTO_2S
#endif

/****** Live timing (ms) ******/
#define DWELL_TIME        5000UL   // Pause at each end (also used by bench continuous runs)
#define MOVE_TIMEOUT      64000UL  // Live only. Relay-era value; re-time on site at final speeds (1.5x slowest trip)
#define LIMIT_CLEAR_TIME  1500UL   // Ship must leave the limit behind it within this time (wrong-way check)
#define FAULT_RETRY_TIME  10000UL  // Cool-down before retry after a fault
#define MAX_FAULTS        3        // Consecutive faults before lockout
#define LIMIT_DEBOUNCE    20UL     // Limit must read steady this long
#define BLINK_NORMAL      1000UL   // LED toggle, running
#define BLINK_FAULT       150UL    // LED toggle, fault

/****** Bench test settings ******/
#define REVERSE_PAUSE       500UL   // Motor braked between calibration legs (ms)
#define JOG_DUTY            255     // Jog speed, 0-255
#define JOG_FIRST_TIMEOUT   700UL   // Jog: wait for key repeat to start (ms). Raise if a held key stutters.
#define JOG_REPEAT_TIMEOUT  200UL   // Jog: stop this long after the last key repeat (ms)
#define STEADY_TOLERANCE    500UL   // Steady calibration: OK within +/- this of target (ms)
#define MIN_OUT_TIME        1000UL  // Min speed: drive out this long at steady duty (ms)
#define MIN_TARGET          3000UL  // Min speed: target return time (ms)
#define MIN_TOLERANCE       250UL   // Min speed: first step at or above MIN_TARGET - this wins
#define MIN_STEP_PCT        10      // Min speed: step down by this % of steady each try

/****** Motion modes ******/
#define MODE_STEADY  0  // Constant duty, full trip
#define MODE_RAMP    1  // S-curve accel from MIN to STEADY, S-curve decel, creep at MIN to the limit

/****** Calibration results — replace this block with bench_test 'p' output ******/
// STEADY_DUTY: PWM 0-255. STEADY_TRIP_MS: measured one-way time at that duty.
// MIN_DUTY: creep duty. MIN_TRIP_MS: estimated one-way time at MIN_DUTY.
// ACCEL_PCT / DECEL_PCT: % of STEADY_TRIP_MS. DECEL_START_MS: decel start, from trip start.
// ---- BEGIN CALIBRATION ----
#define SHIP1_MODE            MODE_STEADY
#define SHIP1_STEADY_DUTY     255
#define SHIP1_STEADY_TRIP_MS  0
#define SHIP1_MIN_DUTY        0
#define SHIP1_MIN_TRIP_MS     0
#define SHIP1_ACCEL_PCT       0
#define SHIP1_DECEL_PCT       0
#define SHIP1_DECEL_START_MS  0
#define SHIP2_MODE            MODE_STEADY
#define SHIP2_STEADY_DUTY     255
#define SHIP2_STEADY_TRIP_MS  0
#define SHIP2_MIN_DUTY        0
#define SHIP2_MIN_TRIP_MS     0
#define SHIP2_ACCEL_PCT       0
#define SHIP2_DECEL_PCT       0
#define SHIP2_DECEL_START_MS  0
// ---- END CALIBRATION ----
