#include "ship_motion.h"

/****** Objects from ship_config.h ******/
ShipIO shipIO[2] = {
  { 1, SHIP1_IN_TO_POS1, SHIP1_IN_TO_POS2,
    { SHIP1_LIMIT_POS1, SHIP1_POS1_ACTIVE, false, false, 0 },
    { SHIP1_LIMIT_POS2, SHIP1_POS2_ACTIVE, false, false, 0 }, -1 },
  { 2, SHIP2_IN_TO_POS1, SHIP2_IN_TO_POS2,
    { SHIP2_LIMIT_POS1, SHIP2_POS1_ACTIVE, false, false, 0 },
    { SHIP2_LIMIT_POS2, SHIP2_POS2_ACTIVE, false, false, 0 }, -1 }
};

const ShipTuning SHIP_TUNING[2] = {
  { SHIP1_MODE, SHIP1_STEADY_DUTY, SHIP1_STEADY_TRIP_MS, SHIP1_MIN_DUTY, SHIP1_MIN_TRIP_MS,
    SHIP1_START_SPEED, SHIP1_RAMP_TO_TAKE_OFF_MS, SHIP1_TAKE_OFF_SPEED, SHIP1_ACCEL_TO_HYPER_SPEED_MS, SHIP1_HYPER_SPEED,
    SHIP1_TIME_AT_HYPER_SPEED_MS, SHIP1_DECEL_TO_CRUISE_MS, SHIP1_CRUISE_SPEED, SHIP1_LIMIT_CLEAR_MS },
  { SHIP2_MODE, SHIP2_STEADY_DUTY, SHIP2_STEADY_TRIP_MS, SHIP2_MIN_DUTY, SHIP2_MIN_TRIP_MS,
    SHIP2_START_SPEED, SHIP2_RAMP_TO_TAKE_OFF_MS, SHIP2_TAKE_OFF_SPEED, SHIP2_ACCEL_TO_HYPER_SPEED_MS, SHIP2_HYPER_SPEED,
    SHIP2_TIME_AT_HYPER_SPEED_MS, SHIP2_DECEL_TO_CRUISE_MS, SHIP2_CRUISE_SPEED, SHIP2_LIMIT_CLEAR_MS }
};

/****** Limits ******/
static void limitBegin(LimitSwitch &sw) {
  pinMode(sw.pin, INPUT_PULLUP);
  sw.lastRaw    = (digitalRead(sw.pin) == sw.activeLevel);
  sw.active     = sw.lastRaw;  // Trust first reading at boot
  sw.lastChange = millis();
}

// Accept a new level only after it has been steady for LIMIT_DEBOUNCE (bounce, long-wire noise).
void limitUpdate(LimitSwitch &sw) {
  bool raw = (digitalRead(sw.pin) == sw.activeLevel);
  if (raw != sw.lastRaw) {
    sw.lastRaw    = raw;
    sw.lastChange = millis();
  } else if (millis() - sw.lastChange >= LIMIT_DEBOUNCE) {
    sw.active = raw;
  }
}

void shipUpdateLimits(ShipIO &s) {
  limitUpdate(s.pos1);
  limitUpdate(s.pos2);
}

bool bothLimits(const ShipIO &s) { return s.pos1.active && s.pos2.active; }
LimitSwitch &aheadOf(ShipIO &s, Dir d)  { return (d == TO_POS1) ? s.pos1 : s.pos2; }
LimitSwitch &behindOf(ShipIO &s, Dir d) { return (d == TO_POS1) ? s.pos2 : s.pos1; }

/****** Motor ******/
void motorBrake(ShipIO &s) {
  digitalWrite(s.inToPos1, HIGH);  // digitalWrite also turns off PWM on that pin (megaavr core)
  digitalWrite(s.inToPos2, HIGH);
  s.lastCmd = -1;
}

/****** PWM frequency ******/
#if   PWM_TCA_DIV == 1
  #define TCA_CLKSEL  TCA_SINGLE_CLKSEL_DIV1_gc
#elif PWM_TCA_DIV == 2
  #define TCA_CLKSEL  TCA_SINGLE_CLKSEL_DIV2_gc
#elif PWM_TCA_DIV == 4
  #define TCA_CLKSEL  TCA_SINGLE_CLKSEL_DIV4_gc
#elif PWM_TCA_DIV == 8
  #define TCA_CLKSEL  TCA_SINGLE_CLKSEL_DIV8_gc
#elif PWM_TCA_DIV == 16
  #define TCA_CLKSEL  TCA_SINGLE_CLKSEL_DIV16_gc
#elif PWM_TCA_DIV != 64
  #error "PWM_TCA_DIV must be 1, 2, 4, 8, 16 or 64"
#endif

// Core default: TCA0 at F_CPU/64 clocks the PWM on D5/D9/D10 (TCA0) and D3 (TCB1, CLKTCA),
// and also the millis() timer TCB3 (8-bit, 256 ticks = 1.024 ms) (core variant.c, wiring.c).
// Speeding up TCA0 would speed up millis(), so TCB3 moves to 16-bit periodic mode with a
// period scaled by the same factor: it still interrupts every 1.024 ms and millis() stays exact.
// micros() and delay() read TCB3's low byte and become wrong; neither is used in this project.
static void pwmBegin() {
#if PWM_TCA_DIV != 64
  uint8_t sreg = SREG;
  cli();
  TCB3.CTRLA &= ~TCB_ENABLE_bm;
  TCB3.CTRLB  = TCB_CNTMODE_INT_gc;
  TCB3.CCMP   = 256U * (64 / PWM_TCA_DIV) - 1;
  TCB3.CNT    = 0;
  TCA0.SINGLE.CTRLA = TCA_CLKSEL | TCA_SINGLE_ENABLE_bm;
  TCB3.CTRLA |= TCB_ENABLE_bm;
  SREG = sreg;
#endif
}

void shipBegin(ShipIO &s) {
  static bool pwmReady = false;  // Timers are shared by both ships: set once
  if (!pwmReady) {
    pwmBegin();
    pwmReady = true;
  }
  // Driver first: pinMode(OUTPUT) gives LOW/LOW = coast, then brake straight away.
  pinMode(s.inToPos1, OUTPUT);
  pinMode(s.inToPos2, OUTPUT);
  motorBrake(s);
  limitBegin(s.pos1);
  limitBegin(s.pos2);
}

void motorDrive(ShipIO &s, Dir d, uint8_t duty) {
  if (duty == 0) {
    motorBrake(s);
    return;
  }
  int16_t cmd = ((d == TO_POS1) ? 0 : 256) + duty;
  if (cmd == s.lastCmd) return;
  s.lastCmd = cmd;

  uint8_t hold = (d == TO_POS1) ? s.inToPos1 : s.inToPos2;
  uint8_t pwm  = (d == TO_POS1) ? s.inToPos2 : s.inToPos1;
  digitalWrite(hold, HIGH);  // Passes through brake (1/1), never the opposite direction
  if (duty == 255) {
    digitalWrite(pwm, LOW);       // Full drive. analogWrite(255) is not a solid level on this core.
  } else {
    analogWrite(pwm, 255 - duty); // PWM HIGH time = brake, LOW time = drive
  }
}

/****** Speed profile ******/
// Smoothstep: 0→1 with zero slope at both ends, so the stages join without jerk.
static float sCurve(float x) { return x * x * (3.0f - 2.0f * x); }

// a -> b along the S-curve, f = 0..1
static uint8_t sBlend(uint8_t a, uint8_t b, float f) {
  return (uint8_t)(a + (b - a) * sCurve(f) + 0.5f);
}

// Start -> take-off, take-off -> hyper, hold hyper, hyper -> cruise, then cruise to the limit. No decel at the end.
static uint8_t surgeDuty(const ShipTuning &t, unsigned long e) {
  uint8_t start = PCT_TO_DUTY(t.startPct), takeOff = PCT_TO_DUTY(t.takeOffPct);
  uint8_t hyper = PCT_TO_DUTY(t.hyperPct), cruise = PCT_TO_DUTY(t.cruisePct);
  if (e < t.takeOffRampMs) return sBlend(start, takeOff, (float)e / t.takeOffRampMs);
  e -= t.takeOffRampMs;
  if (e < t.hyperRampMs) return sBlend(takeOff, hyper, (float)e / t.hyperRampMs);
  e -= t.hyperRampMs;
  if (e < t.hyperHoldMs) return hyper;
  e -= t.hyperHoldMs;
  if (e < t.cruiseRampMs) return sBlend(hyper, cruise, (float)e / t.cruiseRampMs);
  return cruise;
}

// MODE_SURGE starts slow, so it needs its measured time; unmeasured falls back and faults loudly.
unsigned long clearMs(const ShipTuning &t) {
  if (t.mode == MODE_SURGE && t.limitClearMs > 0) return t.limitClearMs;
  return LIMIT_CLEAR_TIME;
}

uint8_t tripDuty(const ShipTuning &t, unsigned long elapsed) {
  if (t.mode == MODE_SURGE) return surgeDuty(t, elapsed);
  return t.steadyDuty;
}
