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
// POS1 = motor end. Remapped 2026-11-05 so each ship's POS1 terminal sits nearest the Nano.
const uint8_t SHIP1_LIMIT_POS1 = 6;
const uint8_t SHIP1_LIMIT_POS2 = 7;
const uint8_t SHIP2_LIMIT_POS1 = 8;
const uint8_t SHIP2_LIMIT_POS2 = 4;

// Pin level when a ship IS at a limit. NO to GND: LOW. NC to GND: HIGH. Bench-verified 2026-10-19: all NC.
#define SHIP1_POS1_ACTIVE  HIGH
#define SHIP1_POS2_ACTIVE  HIGH
#define SHIP2_POS1_ACTIVE  HIGH
#define SHIP2_POS2_ACTIVE  HIGH

/****** PWM frequency ******/
// TCA0 prescaler. 2 = 31.25 kHz (above hearing). 64 = core default 977 Hz (audible squeal).
// DRV8871 allows 0-200 kHz, pulses >= 800 ns (TI SLVSCY9B §6.3). Allowed: 1, 2, 4, 8, 16, 64.
// Changing it changes how speed responds to duty: re-check calibration after a change.
#define PWM_TCA_DIV  2

/****** Status ******/
const uint8_t STATUS_LED = LED_BUILTIN;  // D13: slow blink = OK, fast blink = fault

#ifdef ARDUINO_ARCH_MEGAAVR
  #define WDT_TIMEOUT  WDT_PERIOD_2KCLK_gc  // ATmega4809: ~2 s
#else
  #define WDT_TIMEOUT  WDTO_2S
#endif

/****** Live timing (ms) ******/
#define DWELL_TIME        5000UL   // Pause at each end (also used by bench continuous runs)
#define MOVE_TIMEOUT      93000UL  // Live only. 1.5x slowest bench trip (Ship 2, 62007 ms, 2026-11-05)
#define LIMIT_CLEAR_TIME  1500UL   // Ship must leave the limit behind it within this time (wrong-way check). MODE_SURGE: SHIPx_LIMIT_CLEAR_MS.
#define FAULT_RETRY_TIME  10000UL  // Cool-down before retry after a fault
#define MAX_FAULTS        3        // Consecutive faults before lockout
#define LIMIT_DEBOUNCE    20UL     // Limit must read steady this long
#define BLINK_NORMAL      1000UL   // LED toggle, running
#define BLINK_FAULT       150UL    // LED toggle, fault

/****** Bench test settings ******/
#define REVERSE_PAUSE       500UL   // Motor braked between calibration legs (ms)
#define JOG_PCT             40      // Jog speed at boot, % 1-100. Change at runtime with j.
#define JOG_FIRST_TIMEOUT   700UL   // Jog: wait for key repeat to start (ms). Raise if a held key stutters.
#define JOG_REPEAT_TIMEOUT  200UL   // Jog: stop this long after the last key repeat (ms)
#define STEADY_TOLERANCE    500UL   // Steady calibration: OK within +/- this of target (ms)
#define MIN_OUT_TIME        1000UL  // Min speed: drive out this long at steady duty (ms)
#define MIN_TARGET          3000UL  // Min speed: target return time (ms)
#define MIN_TOLERANCE       250UL   // Min speed: first step at or above MIN_TARGET - this wins
#define MIN_STEP_PCT        10      // Min speed: step down by this % of steady each try

/****** Motion modes ******/
#define MODE_STEADY  0  // Constant duty, full trip
#define MODE_SURGE   1  // S-curves START -> TAKE_OFF -> HYPER, hold HYPER, S-curve -> CRUISE, then CRUISE to the limit

// % 0-100 to PWM duty 0-255, rounded
#define PCT_TO_DUTY(p)  (((p) * 255 + 50) / 100)

/****** Wrong-way time calibration (bench_test c) ******/
// Measures motor-on to start-switch release on the MODE_SURGE profile, from POS1.
#define CLEAR_RUNS         3        // Runs averaged
#define CLEAR_MARGIN_MS    500UL    // Added to the average to get SHIPx_LIMIT_CLEAR_MS
#define CLEAR_RETRACT_ADD_PCT  10   // Back to POS1 at the duty where the switch released + this (% points)
#define CLEAR_CAL_MAX_MS   15000UL  // Bench a/c tests only: wrong-way limit while values are being found

/****** Calibration results — replace this block with bench_test 'p' output ******/
// STEADY_DUTY: PWM 0-255 (115 = 45%, 140 = 55%). STEADY_TRIP_MS: measured one-way time at that duty.
// MIN_DUTY: creep duty. MIN_TRIP_MS: estimated one-way time at MIN_DUTY.
// MODE_SURGE: *_SPEED in % 0-100, times in ms. Set with bench_test a. START_SPEED: above MIN_DUTY, so it moves at once.
// LIMIT_CLEAR_MS: wrong-way time for MODE_SURGE (bench_test c). 0 = not measured: LIMIT_CLEAR_TIME is used.
// ---- BEGIN CALIBRATION ----
#define SHIP1_MODE                    MODE_SURGE
#define SHIP1_STEADY_DUTY             115
#define SHIP1_STEADY_TRIP_MS          41361
#define SHIP1_MIN_DUTY                46
#define SHIP1_MIN_TRIP_MS             134423
#define SHIP1_START_SPEED             25
#define SHIP1_RAMP_TO_TAKE_OFF_MS     10000
#define SHIP1_TAKE_OFF_SPEED          45
#define SHIP1_ACCEL_TO_HYPER_SPEED_MS 2000
#define SHIP1_HYPER_SPEED             90
#define SHIP1_TIME_AT_HYPER_SPEED_MS  10000
#define SHIP1_DECEL_TO_CRUISE_MS      4000
#define SHIP1_CRUISE_SPEED            50
#define SHIP1_LIMIT_CLEAR_MS          757
#define SHIP2_MODE                    MODE_SURGE
#define SHIP2_STEADY_DUTY             150
#define SHIP2_STEADY_TRIP_MS          67365
#define SHIP2_MIN_DUTY                45
#define SHIP2_MIN_TRIP_MS             218936
#define SHIP2_START_SPEED             35
#define SHIP2_RAMP_TO_TAKE_OFF_MS     15000
#define SHIP2_TAKE_OFF_SPEED          50
#define SHIP2_ACCEL_TO_HYPER_SPEED_MS 2000
#define SHIP2_HYPER_SPEED             75
#define SHIP2_TIME_AT_HYPER_SPEED_MS  55000
#define SHIP2_DECEL_TO_CRUISE_MS      2000
#define SHIP2_CRUISE_SPEED            65
#define SHIP2_LIMIT_CLEAR_MS          950
// ---- END CALIBRATION ----
