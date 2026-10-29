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
    SHIP1_ACCEL_PCT, SHIP1_DECEL_PCT, SHIP1_DECEL_START_MS },
  { SHIP2_MODE, SHIP2_STEADY_DUTY, SHIP2_STEADY_TRIP_MS, SHIP2_MIN_DUTY, SHIP2_MIN_TRIP_MS,
    SHIP2_ACCEL_PCT, SHIP2_DECEL_PCT, SHIP2_DECEL_START_MS }
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

void shipBegin(ShipIO &s) {
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
// Smoothstep: 0→1 with zero slope at both ends, so accel and decel have no jerk at the joins.
static float sCurve(float x) { return x * x * (3.0f - 2.0f * x); }

static uint8_t blendDuty(const ShipTuning &t, float f) {
  return t.minDuty + (uint8_t)((t.steadyDuty - t.minDuty) * f + 0.5f);
}

unsigned long accelMs(const ShipTuning &t) { return t.steadyTripMs * t.accelPct / 100; }
unsigned long decelMs(const ShipTuning &t) { return t.steadyTripMs * t.decelPct / 100; }

static float upFrac(const ShipTuning &t, unsigned long e) {
  unsigned long a = accelMs(t);
  if (a == 0 || e >= a) return 1.0f;
  return sCurve((float)e / a);
}

static float downFrac(const ShipTuning &t, unsigned long e) {
  unsigned long ds = t.decelStartMs;
  unsigned long dm = decelMs(t);
  if (e <= ds) return 1.0f;
  if (dm == 0 || e >= ds + dm) return 0.0f;  // Decel done: creep at minDuty until the limit
  return 1.0f - sCurve((float)(e - ds) / dm);
}

uint8_t accelDuty(const ShipTuning &t, unsigned long elapsed) {
  return blendDuty(t, upFrac(t, elapsed));
}

uint8_t tripDuty(const ShipTuning &t, unsigned long elapsed) {
  if (t.mode != MODE_RAMP) return t.steadyDuty;
  float up   = upFrac(t, elapsed);
  float down = downFrac(t, elapsed);
  return blendDuty(t, (up < down) ? up : down);  // Lower wins, so a short trip still decelerates
}
