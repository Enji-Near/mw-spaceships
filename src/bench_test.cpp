#include <Arduino.h>
#include <avr/wdt.h>
#include "ship_motion.h"

/*****  Space Ships — bench test and calibration  *****
 *  115200 baud. Single-key commands; number prompts end with Enter.
 *    5-8  Hold to jog: Ship 1→POS1, Ship 1→POS2, Ship 2→POS1, Ship 2→POS2 (JOG_DUTY). Release to stop.
 *    w    Watch raw limit levels (check NO/NC → SHIPx_POSx_ACTIVE)
 *    t    Steady speed: type a target one-way time per ship, up to 3 round trips, then run
 *    m    Minimum speed: 1000 ms out at steady, return at 90/80/70...% until the return takes >= 2750 ms
 *    r    Ramp: type accel % and decel % per ship, 1 round trip, then run
 *    p    Print the calibration block to paste into include/ship_config.h
 *    s    Stop all;  h  menu
 *  Both ships run together. Enter on an empty prompt skips that ship.
 *  After t and r, both ships run continuously with DWELL_TIME until any key.
 *  No move timeout here (someone is watching): if a ship stalls, press any key.
 *  Kept from live: wrong-way check, both-limits check, watchdog.
 *  Values start from ship_config.h; each test keeps only its latest results.
 *****************************************************/

/****** Types ******/
enum Mode    { IDLE, WATCHING, JOGGING, TASK };
enum Task    { T_STEADY, T_MIN, T_RAMP, T_RUN };
enum Profile { P_CONST, P_ACCEL, P_TRIP };

// One ship's progress through a task
struct Runner {
  bool active;              // Ship takes part in this task
  bool done;
  bool failed;
  uint8_t step;
  // Current move
  bool moving;
  Dir dir;
  Profile profile;
  uint8_t duty;             // For P_CONST
  unsigned long start;
  unsigned long stopAfter;  // 0 = drive to the limit
  unsigned long boostAfter; // 0 = never. Min test: past this, finish the return at steady duty.
  bool boosted;
  bool atLimit;             // Last move ended on the limit (not on stopAfter)
  unsigned long lastMs;     // Last move time, ms
  // Pause before the next step
  bool waiting;
  unsigned long waitStart;
  unsigned long waitMs;
  // Results
  unsigned long target;
  uint8_t run;
  uint8_t runDuty[3];
  unsigned long runMs[3];
  unsigned long legSum;
  uint8_t legs;
  uint8_t pct;
  ShipTuning before;        // Restored if the test fails or is aborted
};

/****** Objects ******/
ShipTuning tune[2];         // Working copy, starts from ship_config.h
Runner rn[2];
Mode mode = IDLE;
Task task = T_RUN;
uint8_t watchRaw[4];

// Jog
bool motorOn[2]             = { false, false };
bool motorUsed[2]           = { false, false };
Dir lastDir[2]              = { TO_POS1, TO_POS1 };
unsigned long motorOffAt[2] = { 0, 0 };
uint8_t jogShip = 0;
Dir jogDir = TO_POS1;
char jogKey = 0;
bool jogRepeating = false;
unsigned long jogLastKey = 0;
char latchedKey = 0;
unsigned long latchedLastAt = 0;

/****** Motor wrappers: track on/off for the jog reverse check and LED ******/
void benchBrake(uint8_t i) {
  motorBrake(shipIO[i]);
  if (motorOn[i]) {
    motorOn[i]    = false;
    motorOffAt[i] = millis();
  }
}

void benchDrive(uint8_t i, Dir d, uint8_t duty) {
  motorOn[i]   = true;
  motorUsed[i] = true;
  lastDir[i]   = d;
  motorDrive(shipIO[i], d, duty);
}

// Failed or aborted test: that ship keeps the values it had before the test.
void restoreUnfinished() {
  if (mode != TASK) return;
  for (uint8_t i = 0; i < 2; i++) {
    if (rn[i].active && (!rn[i].done || rn[i].failed)) tune[i] = rn[i].before;
  }
}

void stopAll() {
  benchBrake(0);
  benchBrake(1);
  rn[0].moving = rn[1].moving = false;
  rn[0].waiting = rn[1].waiting = false;
}

/****** Output Helpers ******/
void printShip(uint8_t i) {
  Serial.print(F("Ship "));
  Serial.print(i + 1);
  Serial.print(F(": "));
}

void printDir(Dir d) { Serial.print((d == TO_POS1) ? F("to POS1") : F("to POS2")); }

void printDefine(uint8_t i, const __FlashStringHelper *name, unsigned long v) {
  Serial.print(F("#define SHIP"));
  Serial.print(i + 1);
  Serial.print(name);
  Serial.print(v);
  Serial.print(F("\n"));
}

void printBlock() {
  Serial.print(F("\n// ---- BEGIN CALIBRATION ----\n"));
  for (uint8_t i = 0; i < 2; i++) {
    Serial.print(F("#define SHIP"));
    Serial.print(i + 1);
    Serial.print((tune[i].mode == MODE_RAMP) ? F("_MODE            MODE_RAMP\n") : F("_MODE            MODE_STEADY\n"));
    printDefine(i, F("_STEADY_DUTY     "), tune[i].steadyDuty);
    printDefine(i, F("_STEADY_TRIP_MS  "), tune[i].steadyTripMs);
    printDefine(i, F("_MIN_DUTY        "), tune[i].minDuty);
    printDefine(i, F("_MIN_TRIP_MS     "), tune[i].minTripMs);
    printDefine(i, F("_ACCEL_PCT       "), tune[i].accelPct);
    printDefine(i, F("_DECEL_PCT       "), tune[i].decelPct);
    printDefine(i, F("_DECEL_START_MS  "), tune[i].decelStartMs);
  }
  Serial.print(F("// ---- END CALIBRATION ----\n"));
}

void printMenu() {
  Serial.print(F("\n--- Space Ships bench test v0.4 ---\n"));
  Serial.print(F("5-8 hold to jog: S1>POS1, S1>POS2, S2>POS1, S2>POS2\n"));
  Serial.print(F("w  watch raw limit levels\n"));
  Serial.print(F("t  steady speed calibration, then run\n"));
  Serial.print(F("m  minimum speed calibration\n"));
  Serial.print(F("r  ramp calibration, then run\n"));
  Serial.print(F("p  print calibration block\n"));
  Serial.print(F("s  stop all\n"));
  Serial.print(F("h  this menu\n"));
  Serial.print(F("Any key stops a running test.\n> "));
}

void toIdle() {
  stopAll();
  mode = IDLE;
  Serial.print(F("> "));
}

void abortAll(int8_t ship, const __FlashStringHelper *reason) {
  restoreUnfinished();
  stopAll();
  mode = IDLE;
  Serial.print(F("\nABORT: "));
  if (ship >= 0) printShip(ship);
  Serial.print(reason);
  Serial.print(F("\n> "));
}

/****** Number Prompt ******/
// Blocking is fine here: motors are braked while waiting. Returns -1 = cancelled, -2 = empty (skip).
long readNumber(const __FlashStringHelper *prompt) {
  Serial.print(prompt);
  long v = 0;
  bool any = false;
  while (true) {
    wdt_reset();
    if (!Serial.available()) continue;
    char c = Serial.read();
    if (c >= '0' && c <= '9') {
      if (v > 999999L) continue;  // Ignore absurd lengths
      v = v * 10 + (c - '0');
      any = true;
      Serial.print(c);
    } else if (c == '\r' || c == '\n') {
      // Swallow the rest of a CR+LF pair, so it can't answer the next prompt.
      unsigned long t = millis();
      while (millis() - t < 20) {
        int n = Serial.peek();
        if (n == '\r' || n == '\n') Serial.read();
      }
      Serial.print(F("\n"));
      return any ? v : -2;
    } else {
      Serial.print(F("\nCancelled\n"));
      return -1;
    }
  }
}

/****** Task Runner ******/
void startMove(uint8_t i, Dir d, Profile p, uint8_t duty) {
  Runner &r = rn[i];
  r.dir        = d;
  r.profile    = p;
  r.duty       = duty;
  r.start      = millis();
  r.stopAfter  = 0;
  r.boostAfter = 0;
  r.boosted    = false;
  r.moving     = true;
}

void waitThen(uint8_t i, unsigned long ms) {
  rn[i].waiting   = true;
  rn[i].waitStart = millis();
  rn[i].waitMs    = ms;
}

void shipDone(uint8_t i, bool ok) {
  rn[i].done   = true;
  rn[i].failed = !ok;
}

Dir awayFromLimit(uint8_t i) { return shipIO[i].pos1.active ? TO_POS2 : TO_POS1; }
bool atALimit(uint8_t i)     { return shipIO[i].pos1.active || shipIO[i].pos2.active; }

uint8_t currentDuty(uint8_t i, unsigned long e) {
  Runner &r = rn[i];
  switch (r.profile) {
    case P_ACCEL: return accelDuty(tune[i], e);
    case P_TRIP:  return tripDuty(tune[i], e);
    default:      return r.boosted ? tune[i].steadyDuty : r.duty;
  }
}

// Time from motor ON to the first raw edge of the ahead switch, not the debounced edge.
unsigned long arrivalMs(const LimitSwitch &ahead, unsigned long start) {
  long t = (long)(ahead.lastChange - start);
  return (t > 0) ? (unsigned long)t : millis() - start;
}

void printLeg(uint8_t i) {
  printShip(i);
  printDir(rn[i].dir);
  Serial.print(F(" "));
  Serial.print(rn[i].lastMs);
  Serial.print(F(" ms\n"));
}

/****** Steady speed (t) ******/
uint8_t clampDuty(float d) {
  if (d < 1.0f) return 1;
  if (d > 255.0f) return 255;
  return (uint8_t)(d + 0.5f);
}

// Run 2: assume speed proportional to duty. Run 3: straight line through runs 1 and 2 (speed = 1/time).
uint8_t nextSteadyDuty(Runner &r) {
  if (r.run == 0) return clampDuty(255.0f * r.runMs[0] / r.target);
  float v0 = 1.0f / r.runMs[0], v1 = 1.0f / r.runMs[1], vt = 1.0f / r.target;
  if (v0 <= v1) return r.runDuty[1];  // No usable slope: keep run 2
  return clampDuty(r.runDuty[1] + (vt - v1) * (r.runDuty[0] - r.runDuty[1]) / (v0 - v1));
}

void finishSteady(uint8_t i, uint8_t k) {
  Runner &r = rn[i];
  tune[i].steadyDuty   = r.runDuty[k];
  tune[i].steadyTripMs = r.runMs[k];
  tune[i].mode         = MODE_STEADY;  // Run after t is steady; re-run r for ramp
  printShip(i);
  Serial.print(F("steady duty "));
  Serial.print(r.runDuty[k]);
  Serial.print(F(", trip "));
  Serial.print(r.runMs[k]);
  Serial.print(F(" ms (target "));
  Serial.print(r.target);
  Serial.print(F(")"));
  long err = (long)r.runMs[k] - (long)r.target;
  if (k == 0) Serial.print(F(" full power"));
  if (k == 0 && err > (long)STEADY_TOLERANCE) Serial.print(F(", target is faster than full power"));
  if (k > 0 && abs(err) > (long)STEADY_TOLERANCE) Serial.print(F(" OUTSIDE TOLERANCE"));
  Serial.print(F("\n"));
  shipDone(i, true);
}

void stepSteady(uint8_t i) {
  Runner &r = rn[i];
  switch (r.step) {
    case 0:
      r.run = 0;
      r.runDuty[0] = 255;
      r.legs = 0;
      r.legSum = 0;
      r.step = 1;
      if (!atALimit(i)) {
        printShip(i);
        Serial.print(F("homing to POS1 (untimed)\n"));
        startMove(i, TO_POS1, P_CONST, 255);
        r.step = 3;  // Homing done → pause → run 1
        return;
      }
      [[fallthrough]];  // Already at a limit
    case 1:
      startMove(i, awayFromLimit(i), P_CONST, r.runDuty[r.run]);
      r.step = 2;
      return;
    case 2: {
      r.legSum += r.lastMs;
      r.legs++;
      printShip(i);
      Serial.print(F("run "));
      Serial.print(r.run + 1);
      Serial.print(F(" duty "));
      Serial.print(r.runDuty[r.run]);
      Serial.print(F(", "));
      printDir(r.dir);
      Serial.print(F(" "));
      Serial.print(r.lastMs);
      Serial.print(F(" ms\n"));
      if (r.legs < 2) {
        r.step = 1;
        waitThen(i, REVERSE_PAUSE);
        return;
      }
      r.runMs[r.run] = r.legSum / 2;
      long err = (long)r.runMs[r.run] - (long)r.target;
      bool ok = abs(err) <= (long)STEADY_TOLERANCE;
      if (r.run == 0 && err >= -(long)STEADY_TOLERANCE) { finishSteady(i, 0); return; }  // Full power already slow enough
      if (ok || r.run == 2) { finishSteady(i, r.run); return; }
      uint8_t next = nextSteadyDuty(r);
      r.run++;
      r.runDuty[r.run] = next;
      r.legs = 0;
      r.legSum = 0;
      r.step = 1;
      waitThen(i, REVERSE_PAUSE);
      return;
    }
    case 3:
      r.step = 1;
      waitThen(i, REVERSE_PAUSE);
      return;
  }
}

/****** Minimum speed (m) ******/
void stepMin(uint8_t i) {
  Runner &r = rn[i];
  switch (r.step) {
    case 0:
      r.pct = 100 - MIN_STEP_PCT;
      r.step = 1;
      if (!shipIO[i].pos1.active) {
        printShip(i);
        Serial.print(F("homing to POS1 (untimed)\n"));
        startMove(i, TO_POS1, P_CONST, tune[i].steadyDuty);
        r.step = 2;  // Homing done → pause → out
        return;
      }
      [[fallthrough]];
    case 1:  // Out toward POS2 for MIN_OUT_TIME
      startMove(i, TO_POS2, P_CONST, tune[i].steadyDuty);
      r.stopAfter = MIN_OUT_TIME;
      r.step = 2;
      return;
    case 2:
      if (r.dir == TO_POS1) {  // Homing just finished
        r.step = 1;
        waitThen(i, REVERSE_PAUSE);
        return;
      }
      if (r.atLimit) {
        printShip(i);
        Serial.print(F("reached POS2 within MIN_OUT_TIME, line too short for this test\n"));
        shipDone(i, false);
        return;
      }
      if (shipIO[i].pos1.active) {
        printShip(i);
        Serial.print(F("still on POS1 after MIN_OUT_TIME, ship did not leave\n"));
        shipDone(i, false);
        return;
      }
      r.step = 3;
      waitThen(i, REVERSE_PAUSE);
      return;
    case 3: {  // Return at pct of steady
      uint8_t d = (uint8_t)((unsigned int)tune[i].steadyDuty * r.pct / 100);
      if (d == 0) d = 1;
      startMove(i, TO_POS1, P_CONST, d);
      r.boostAfter = MIN_TARGET + MIN_TOLERANCE;
      r.step = 4;
      return;
    }
    case 4: {
      unsigned long t = r.boosted ? r.boostAfter : r.lastMs;
      printShip(i);
      Serial.print(r.pct);
      Serial.print(F("% (duty "));
      Serial.print(r.duty);
      Serial.print(F(") return "));
      if (r.boosted) Serial.print(F("> "));
      Serial.print(t);
      Serial.print(F(" ms\n"));
      if (r.boosted || t >= MIN_TARGET - MIN_TOLERANCE) {
        tune[i].minDuty = r.duty;
        // Full trip time at minDuty, from the speed ratio. Stays valid if steady is re-run later.
        tune[i].minTripMs = tune[i].steadyTripMs * t / MIN_OUT_TIME;
        printShip(i);
        Serial.print(F("min duty "));
        Serial.print(tune[i].minDuty);
        if (r.boosted) Serial.print(F(" — return did not finish at this duty, may be too slow to move. Check it moved."));
        Serial.print(F("\n"));
        shipDone(i, true);
        return;
      }
      if (r.pct <= MIN_STEP_PCT) {
        printShip(i);
        Serial.print(F("no step reached the target return time\n"));
        shipDone(i, false);
        return;
      }
      r.pct -= MIN_STEP_PCT;
      r.step = 1;
      waitThen(i, REVERSE_PAUSE);
      return;
    }
  }
}

/****** Ramp (r) ******/
void finishRamp(uint8_t i) {
  Runner &r = rn[i];
  ShipTuning &t = tune[i];
  unsigned long accelTrip = r.legSum / 2;
  // Decel from steady to min averages (vSteady + vMin) / 2 on an S-curve, so it covers
  // decelMs * (1 + ratio) / 2 of steady-speed travel time. Start it that much before the accel-trip arrival.
  float ratio = (float)t.steadyTripMs / t.minTripMs;  // vMin / vSteady
  float lead  = decelMs(t) * (1.0f + ratio) / 2.0f;
  long start  = (long)accelTrip - (long)(lead + 0.5f);
  printShip(i);
  Serial.print(F("accel trip avg "));
  Serial.print(accelTrip);
  Serial.print(F(" ms, decel start "));
  if (start < 0) {
    start = 0;
    Serial.print(F("0 ms (DECEL_PCT too large for this trip)"));
  } else {
    Serial.print(start);
    Serial.print(F(" ms"));
  }
  Serial.print(F("\n"));
  t.decelStartMs = (unsigned long)start;
  t.mode = MODE_RAMP;
  shipDone(i, true);
}

void stepRamp(uint8_t i) {
  Runner &r = rn[i];
  switch (r.step) {
    case 0:
      r.legs = 0;
      r.legSum = 0;
      r.step = 1;
      if (!shipIO[i].pos1.active) {
        printShip(i);
        Serial.print(F("homing to POS1 (untimed)\n"));
        startMove(i, TO_POS1, P_CONST, tune[i].steadyDuty);
        r.step = 3;  // Homing done → pause → first leg
        return;
      }
      [[fallthrough]];
    case 1:
      startMove(i, awayFromLimit(i), P_ACCEL, 0);
      r.step = 2;
      return;
    case 2:
      r.legSum += r.lastMs;
      r.legs++;
      printShip(i);
      Serial.print(F("accel only, "));
      printDir(r.dir);
      Serial.print(F(" "));
      Serial.print(r.lastMs);
      Serial.print(F(" ms\n"));
      if (r.legs < 2) {
        r.step = 1;
        waitThen(i, REVERSE_PAUSE);
        return;
      }
      finishRamp(i);
      return;
    case 3:
      r.step = 1;
      waitThen(i, REVERSE_PAUSE);
      return;
  }
}

/****** Continuous run (after t and r) ******/
void stepRun(uint8_t i) {
  Runner &r = rn[i];
  switch (r.step) {
    case 0:
      r.step = 1;
      if (atALimit(i)) {
        waitThen(i, DWELL_TIME);
      } else {
        startMove(i, TO_POS1, P_TRIP, 0);  // Mid-track: home to POS1, same as live
        r.step = 2;
      }
      return;
    case 1:
      startMove(i, awayFromLimit(i), P_TRIP, 0);
      r.step = 2;
      return;
    case 2:
      printLeg(i);
      r.step = 1;
      waitThen(i, DWELL_TIME);
      return;
  }
}

void stepTask(uint8_t i) {
  switch (task) {
    case T_STEADY: stepSteady(i); break;
    case T_MIN:    stepMin(i); break;
    case T_RAMP:   stepRamp(i); break;
    case T_RUN:    stepRun(i); break;
  }
}

void endMove(uint8_t i, unsigned long ms, bool atLimit) {
  benchBrake(i);
  rn[i].moving  = false;
  rn[i].lastMs  = ms;
  rn[i].atLimit = atLimit;
  stepTask(i);
}

void updateRunner(uint8_t i) {
  Runner &r = rn[i];
  if (r.done) return;

  if (r.waiting) {
    if (millis() - r.waitStart >= r.waitMs) {
      r.waiting = false;
      stepTask(i);
    }
    return;
  }
  if (!r.moving) return;

  ShipIO &s = shipIO[i];
  LimitSwitch &ahead = aheadOf(s, r.dir);
  unsigned long e = millis() - r.start;

  if (bothLimits(s)) {
    abortAll(i, F("both limits active — check wiring"));
  } else if (ahead.active) {
    endMove(i, arrivalMs(ahead, r.start), true);
  } else if (behindOf(s, r.dir).active && e >= LIMIT_CLEAR_TIME) {
    abortAll(i, F("limit behind ship still active — motor reversed, jammed, or too slow to clear in LIMIT_CLEAR_TIME"));
  } else if (r.stopAfter && e >= r.stopAfter) {
    endMove(i, e, false);
  } else {
    if (r.boostAfter && !r.boosted && e >= r.boostAfter) r.boosted = true;
    benchDrive(i, r.dir, currentDuty(i, e));
  }
}

void startTask(Task t, bool use1, bool use2) {
  bool use[2] = { use1, use2 };
  for (uint8_t i = 0; i < 2; i++) {
    if (use[i] && bothLimits(shipIO[i])) {
      abortAll(i, F("both limits active — check wiring"));
      return;
    }
  }
  task = t;
  mode = TASK;
  for (uint8_t i = 0; i < 2; i++) {
    memset(&rn[i], 0, sizeof(Runner));
    rn[i].before = tune[i];
    rn[i].active = use[i];
    rn[i].done   = !use[i];
  }
}

// Calls the first step after the targets are stored (startTask clears the runners).
void kickTask() {
  for (uint8_t i = 0; i < 2; i++) if (rn[i].active) stepTask(i);
}

void startRun() {
  Serial.print(F("\nRunning both ships with DWELL_TIME. Any key stops.\n"));
  startTask(T_RUN, true, true);
  if (mode == TASK) kickTask();
}

// Called when every active ship is done.
void taskFinished() {
  bool anyFail = (rn[0].active && rn[0].failed) || (rn[1].active && rn[1].failed);
  restoreUnfinished();
  stopAll();
  mode = IDLE;
  if (anyFail) {
    Serial.print(F("\nCalibration failed for a ship, see above. Values for that ship unchanged.\n"));
  }
  printBlock();
  if (!anyFail && (task == T_STEADY || task == T_RAMP)) {
    startRun();
  } else {
    Serial.print(F("> "));
  }
}

/****** Commands t / m / r ******/
void cmdSteady() {
  long tgt[2];
  for (uint8_t i = 0; i < 2; i++) {
    printShip(i);
    tgt[i] = readNumber(F("target one-way time, ms (Enter = skip): "));
    if (tgt[i] == -1) { Serial.print(F("> ")); return; }
    if (tgt[i] == 0) { Serial.print(F("Must be > 0\n> ")); return; }
  }
  if (tgt[0] < 0 && tgt[1] < 0) { Serial.print(F("Both skipped\n> ")); return; }
  startTask(T_STEADY, tgt[0] > 0, tgt[1] > 0);
  if (mode != TASK) return;
  for (uint8_t i = 0; i < 2; i++) rn[i].target = (tgt[i] > 0) ? tgt[i] : 0;
  Serial.print(F("Steady calibration. Run 1 at full power. Any key aborts.\n"));
  kickTask();
}

bool steadyReady(uint8_t i) { return tune[i].steadyTripMs > 0; }
bool minReady(uint8_t i)    { return tune[i].minDuty > 0 && tune[i].minTripMs > 0 && tune[i].minDuty < tune[i].steadyDuty; }

void cmdMin() {
  for (uint8_t i = 0; i < 2; i++) {
    if (!steadyReady(i)) {
      printShip(i);
      Serial.print(F("no steady values, run t first\n> "));
      return;
    }
  }
  startTask(T_MIN, true, true);
  if (mode != TASK) return;
  Serial.print(F("Minimum speed calibration. Any key aborts.\n"));
  kickTask();
}

void cmdRamp() {
  long acc[2] = { -2, -2 }, dec[2] = { -2, -2 };
  for (uint8_t i = 0; i < 2; i++) {
    printShip(i);
    acc[i] = readNumber(F("accel, % of trip (Enter = skip ship): "));
    if (acc[i] == -1) { Serial.print(F("> ")); return; }
    if (acc[i] == -2) continue;
    if (!steadyReady(i) || !minReady(i)) {
      printShip(i);
      Serial.print(F("needs steady (t) and minimum speed (m) first\n> "));
      return;
    }
    printShip(i);
    dec[i] = readNumber(F("decel, % of trip: "));
    if (dec[i] < 0) { Serial.print(F("> ")); return; }
    if (acc[i] + dec[i] > 100) { Serial.print(F("accel + decel must be <= 100\n> ")); return; }
  }
  if (acc[0] < 0 && acc[1] < 0) { Serial.print(F("Both skipped\n> ")); return; }
  startTask(T_RAMP, acc[0] >= 0, acc[1] >= 0);
  if (mode != TASK) return;
  for (uint8_t i = 0; i < 2; i++) {
    if (acc[i] < 0) continue;
    tune[i].accelPct = (uint8_t)acc[i];
    tune[i].decelPct = (uint8_t)dec[i];
  }
  Serial.print(F("Ramp calibration: 1 round trip with accel only. Any key aborts.\n"));
  kickTask();
}

/****** Limit Watch ******/
const uint8_t WATCH_PINS[4]   = { SHIP1_LIMIT_POS1, SHIP1_LIMIT_POS2, SHIP2_LIMIT_POS1, SHIP2_LIMIT_POS2 };
const uint8_t WATCH_ACTIVE[4] = { SHIP1_POS1_ACTIVE, SHIP1_POS2_ACTIVE, SHIP2_POS1_ACTIVE, SHIP2_POS2_ACTIVE };

void printWatchLine() {
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F("S"));
    Serial.print(i / 2 + 1);
    Serial.print((i % 2) ? F("P2") : F("P1"));
    Serial.print(watchRaw[i] == HIGH ? F("=HIGH") : F("=LOW "));
    Serial.print(watchRaw[i] == WATCH_ACTIVE[i] ? F("* ") : F("  "));
  }
  Serial.print(F("\n"));
}

void readWatchPins() {
  for (uint8_t i = 0; i < 4; i++) watchRaw[i] = digitalRead(WATCH_PINS[i]);
}

void startWatch() {
  mode = WATCHING;
  Serial.print(F("Raw pin levels, printed on change. * = active with current SHIPx_POSx_ACTIVE.\n"));
  Serial.print(F("Ship away from switch: HIGH = NO to GND, LOW = NC to GND. Any key exits.\n"));
  readWatchPins();
  printWatchLine();
}

void updateWatch() {
  uint8_t prev[4];
  memcpy(prev, watchRaw, sizeof(prev));
  readWatchPins();
  if (memcmp(prev, watchRaw, sizeof(prev)) != 0) printWatchLine();
}

/****** Jog (hold 5-8) ******/
// A terminal sends no key-release: the motor runs only while key repeats keep arriving (dead-man).
void latchKey(char c) {
  latchedKey    = c;
  latchedLastAt = millis();
}

// Ignore a held key until released, so repeats can't restart a jog that just stopped.
bool isLatched(char c) {
  if (c != latchedKey) return false;
  if (millis() - latchedLastAt >= JOG_REPEAT_TIMEOUT) {
    latchedKey = 0;
    return false;
  }
  latchedLastAt = millis();
  return true;
}

bool reverseTooSoon(uint8_t i, Dir d) {
  if (!motorUsed[i] || lastDir[i] == d) return false;
  return motorOn[i] || (millis() - motorOffAt[i] < REVERSE_PAUSE);
}

void refuseJog(uint8_t i, Dir d, const __FlashStringHelper *why, char key) {
  printShip(i);
  printDir(d);
  Serial.print(F(" "));
  Serial.print(why);
  Serial.print(F("\n> "));
  latchKey(key);
}

void startJog(uint8_t i, Dir d, char key) {
  ShipIO &s = shipIO[i];
  if (bothLimits(s))         { abortAll(i, F("both limits active — check wiring")); latchKey(key); return; }
  if (aheadOf(s, d).active)  { refuseJog(i, d, F("already at that limit"), key); return; }
  if (reverseTooSoon(i, d))  { refuseJog(i, d, F("motor just stopped, wait REVERSE_PAUSE"), key); return; }
  jogShip      = i;
  jogDir       = d;
  jogKey       = key;
  jogRepeating = false;
  jogLastKey   = millis();
  rn[i].start  = millis();
  mode         = JOGGING;
  printShip(i);
  printDir(d);
  Serial.print(F(" jogging, release key to stop\n"));
  benchDrive(i, d, JOG_DUTY);
}

void endJog(const __FlashStringHelper *reason) {
  Serial.print(reason);
  Serial.print(F("\n"));
  latchKey(jogKey);
  toIdle();
}

void updateJog() {
  ShipIO &s = shipIO[jogShip];
  if (bothLimits(s)) {
    abortAll(jogShip, F("both limits active — check wiring"));
    latchKey(jogKey);
  } else if (aheadOf(s, jogDir).active) {
    endJog(F("Reached limit, stopped"));
  } else if (behindOf(s, jogDir).active && millis() - rn[jogShip].start >= LIMIT_CLEAR_TIME) {
    abortAll(jogShip, F("limit behind ship still active — motor reversed or ship jammed"));
    latchKey(jogKey);
  } else if (millis() - jogLastKey >= (jogRepeating ? JOG_REPEAT_TIMEOUT : JOG_FIRST_TIMEOUT)) {
    endJog(F("Key released, stopped"));
  }
}

/****** Serial Commands ******/
void handleIdleKey(char c) {
  switch (c) {
    case '5': startJog(0, TO_POS1, c); break;
    case '6': startJog(0, TO_POS2, c); break;
    case '7': startJog(1, TO_POS1, c); break;
    case '8': startJog(1, TO_POS2, c); break;
    case 'w': startWatch(); break;
    case 't': cmdSteady(); break;
    case 'm': cmdMin(); break;
    case 'r': cmdRamp(); break;
    case 'p': printBlock(); Serial.print(F("> ")); break;
    case 's': stopAll(); Serial.print(F("All stopped\n> ")); break;
    case 'h':
    case '?': printMenu(); break;
    default:
      Serial.print(F("Unknown key '"));
      Serial.print(c);
      Serial.print(F("', h for menu\n> "));
      break;
  }
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\r' || c == '\n') return;

  if (mode == JOGGING) {
    if (c == jogKey) {  // Key still held: keep the dead-man alive
      jogRepeating = true;
      jogLastKey   = millis();
    } else {
      endJog(F("Jog stopped by key"));
    }
  } else if (mode == WATCHING) {
    Serial.print(F("Watch ended\n"));
    toIdle();
  } else if (mode == TASK) {
    Serial.print(F("\nStopped by key\n"));
    restoreUnfinished();
    printBlock();
    toIdle();
  } else if (!isLatched(c)) {
    handleIdleKey(c);
  }
}

/****** Status LED: ON while any motor is driven ******/
void updateStatusLED() {
  digitalWrite(STATUS_LED, (motorOn[0] || motorOn[1]) ? HIGH : LOW);
}

/****** Setup ******/
void setup() {
  shipBegin(shipIO[0]);  // Drivers braked first
  shipBegin(shipIO[1]);
  tune[0] = SHIP_TUNING[0];
  tune[1] = SHIP_TUNING[1];
  pinMode(STATUS_LED, OUTPUT);
  Serial.begin(115200);
  printMenu();
  wdt_enable(WDT_TIMEOUT);
}

/****** Main Loop ******/
void loop() {
  wdt_reset();
  shipUpdateLimits(shipIO[0]);
  shipUpdateLimits(shipIO[1]);
  handleSerial();

  switch (mode) {
    case JOGGING:  updateJog(); break;
    case WATCHING: updateWatch(); break;
    case TASK:
      updateRunner(0);
      updateRunner(1);
      if (mode == TASK && rn[0].done && rn[1].done) taskFinished();
      break;
    case IDLE: break;
  }
  updateStatusLED();
}
