#include <Arduino.h>
#include <avr/wdt.h>
#include "ship_motion.h"

/*****  Space Ships — v0.4 live firmware  *****
 *  Platform: Arduino Nano Every (ATmega4809, 5V logic). One DRV8871 per ship.
 *  Cycle per ship (independent): move → hit limit → brake → DWELL_TIME → reverse, repeating.
 *  Speed per ship from ship_config.h: MODE_STEADY or MODE_RAMP. Fixed values, nothing adapts.
 *  All tunable values: include/ship_config.h. Motor and limit code: ship_motion.cpp (shared).
 *
 *  Safety layers:
 *    1. Move timeout — target limit never reached (slipped rope, jam, dead switch)
 *    2. Wrong-way check — limit behind the ship still active after LIMIT_CLEAR_TIME
 *    3. Both-limits check — mechanically impossible → wiring fault
 *    4. Fault → brake, cool-down, retry; lockout after MAX_FAULTS in a row (power cycle clears)
 *    5. Hardware watchdog (~2 s). Reset = driver inputs low = coast.
 *********************************************/

// Ramp needs calibrated values: fail the build, not the display.
#if (SHIP1_MODE == MODE_RAMP) && (SHIP1_STEADY_TRIP_MS == 0 || SHIP1_MIN_DUTY == 0 || SHIP1_MIN_DUTY >= SHIP1_STEADY_DUTY)
  #error "Ship 1 RAMP: calibration values missing. Run bench_test t, m, r, then paste 'p' output."
#endif
#if (SHIP2_MODE == MODE_RAMP) && (SHIP2_STEADY_TRIP_MS == 0 || SHIP2_MIN_DUTY == 0 || SHIP2_MIN_DUTY >= SHIP2_STEADY_DUTY)
  #error "Ship 2 RAMP: calibration values missing. Run bench_test t, m, r, then paste 'p' output."
#endif

/****** Types ******/
enum ShipState {
  MOVING_TO_POS1,
  DWELL_AT_POS1,
  MOVING_TO_POS2,
  DWELL_AT_POS2,
  FAULT,        // Braked, waiting FAULT_RETRY_TIME, then retry
  LOCKED_OUT    // Braked until power cycle
};

struct Ship {
  ShipIO &io;
  const ShipTuning &tune;
  ShipState state;
  ShipState lastMove;       // Last direction of travel, for the retry direction
  unsigned long stateStart;
  uint8_t faultCount;       // Consecutive faults, reset on every arrival
};

Ship ships[2] = {
  { shipIO[0], SHIP_TUNING[0], LOCKED_OUT, MOVING_TO_POS1, 0, 0 },
  { shipIO[1], SHIP_TUNING[1], LOCKED_OUT, MOVING_TO_POS1, 0, 0 }
};

/****** Debug Output ******/
void debugState(const Ship &s) {
  #if DEBUG
  Serial.print(F("Ship "));
  Serial.print(s.io.id);
  switch (s.state) {
    case MOVING_TO_POS1: Serial.print(F(": moving to position 1\n")); break;
    case DWELL_AT_POS1:  Serial.print(F(": at position 1, dwelling\n")); break;
    case MOVING_TO_POS2: Serial.print(F(": moving to position 2\n")); break;
    case DWELL_AT_POS2:  Serial.print(F(": at position 2, dwelling\n")); break;
    case FAULT:          Serial.print(F(": FAULT — retry after cool-down\n")); break;
    case LOCKED_OUT:     Serial.print(F(": LOCKED OUT — power cycle to clear\n")); break;
  }
  #endif
}

/****** State Machine ******/
Dir moveDir(ShipState st) { return (st == MOVING_TO_POS1) ? TO_POS1 : TO_POS2; }

// Single entry point for every state change, so outputs and state stay in sync.
void enterState(Ship &s, ShipState next) {
  s.state      = next;
  s.stateStart = millis();
  if (next == MOVING_TO_POS1 || next == MOVING_TO_POS2) {
    s.lastMove = next;
    motorDrive(s.io, moveDir(next), tripDuty(s.tune, 0));
  } else {
    motorBrake(s.io);  // Dwell, fault, lockout
  }
  debugState(s);
}

void shipFault(Ship &s, const __FlashStringHelper *reason) {
  s.faultCount++;
  #if DEBUG
  Serial.print(F("Ship "));
  Serial.print(s.io.id);
  Serial.print(F(" fault "));
  Serial.print(s.faultCount);
  Serial.print(F("/"));
  Serial.print(MAX_FAULTS);
  Serial.print(F(": "));
  Serial.print(reason);
  Serial.print(F("\n"));
  #endif
  enterState(s, (s.faultCount >= MAX_FAULTS) ? LOCKED_OUT : FAULT);
}

// At a limit: head for the other end. Mid-track: go back the way it came (that path last worked).
void shipRecover(Ship &s) {
  if (s.io.pos1.active)      enterState(s, MOVING_TO_POS2);
  else if (s.io.pos2.active) enterState(s, MOVING_TO_POS1);
  else enterState(s, (s.lastMove == MOVING_TO_POS1) ? MOVING_TO_POS2 : MOVING_TO_POS1);
}

void updateMove(Ship &s, ShipState arrivedState) {
  Dir d = moveDir(s.state);
  unsigned long elapsed = millis() - s.stateStart;

  if (aheadOf(s.io, d).active) {
    s.faultCount = 0;
    enterState(s, arrivedState);
  } else if (behindOf(s.io, d).active && elapsed >= LIMIT_CLEAR_TIME) {
    shipFault(s, F("limit behind ship still active — motor reversed or ship jammed"));
  } else if (elapsed >= MOVE_TIMEOUT) {
    shipFault(s, F("move timeout — limit never reached"));
  } else {
    motorDrive(s.io, d, tripDuty(s.tune, elapsed));  // Only rewrites PWM when duty changes
  }
}

void updateShip(Ship &s) {
  shipUpdateLimits(s.io);

  // Skipped in FAULT/LOCKED_OUT: already braked there.
  if (s.state != FAULT && s.state != LOCKED_OUT && bothLimits(s.io)) {
    shipFault(s, F("both limits active — check wiring"));
    return;
  }

  unsigned long elapsed = millis() - s.stateStart;
  switch (s.state) {
    case MOVING_TO_POS1: updateMove(s, DWELL_AT_POS1); break;
    case MOVING_TO_POS2: updateMove(s, DWELL_AT_POS2); break;
    case DWELL_AT_POS1:  if (elapsed >= DWELL_TIME) enterState(s, MOVING_TO_POS2); break;
    case DWELL_AT_POS2:  if (elapsed >= DWELL_TIME) enterState(s, MOVING_TO_POS1); break;
    case FAULT:          if (elapsed >= FAULT_RETRY_TIME) shipRecover(s); break;
    case LOCKED_OUT:     break;
  }
}

/****** Ship Startup ******/
void shipStart(Ship &s) {
  shipBegin(s.io);  // Driver braked first, then limits read
  s.faultCount = 0;

  // Position unknown after power-up: work it out from the limits.
  if (bothLimits(s.io))      shipFault(s, F("both limits active at boot — check wiring"));
  else if (s.io.pos1.active) enterState(s, DWELL_AT_POS1);
  else if (s.io.pos2.active) enterState(s, DWELL_AT_POS2);
  else                       enterState(s, MOVING_TO_POS1);  // Mid-track: home to POS1
}

/****** Status LED ******/
bool isFaulted(const Ship &s) { return (s.state == FAULT || s.state == LOCKED_OUT); }

void updateStatusLED() {
  static unsigned long lastToggle = 0;
  static bool ledOn = false;
  unsigned long interval = (isFaulted(ships[0]) || isFaulted(ships[1])) ? BLINK_FAULT : BLINK_NORMAL;
  if (millis() - lastToggle >= interval) {
    lastToggle = millis();
    ledOn = !ledOn;
    digitalWrite(STATUS_LED, ledOn ? HIGH : LOW);
  }
}

/****** Setup ******/
void setup() {
  #if DEBUG
  Serial.begin(115200);
  Serial.print(F("Space Ships v0.4: Online\n"));
  #endif
  pinMode(STATUS_LED, OUTPUT);
  shipStart(ships[0]);
  shipStart(ships[1]);
  wdt_enable(WDT_TIMEOUT);  // Last, after setup. Loop is non-blocking.
}

/****** Main Loop ******/
void loop() {
  wdt_reset();
  updateShip(ships[0]);
  updateShip(ships[1]);
  updateStatusLED();
}
