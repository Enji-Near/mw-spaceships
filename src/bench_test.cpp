#include <Arduino.h>
#include <limits.h>
#include <avr/wdt.h>
#include "ship_hardware.h"

/*****  Space Ships — bench test  *****
 *  Serial menu for bench setup: 115200 baud, single-key commands (no Enter needed).
 *    1-4  Click one relay for CLICK_TIME (check RELAY_ON polarity)
 *    w    Watch raw limit levels, print on every change (check NO/NC → LIMIT_ACTIVE)
 *    a/b  Time ship 1/2: home to POS1, then ROUND_TRIPS round trips, print averages
 *    s    Stop all motors
 *    h    Menu
 *  Any key during a click, watch, or timing run stops it.
 *
 *  WARNING: clicks and timing runs drive the motors.
 *  Same safety checks as the live firmware: move timeout, wrong-way, both-limits, watchdog.
 *  No DEBUG switch: serial output is this firmware's whole job.
 **************************************/

/****** Bench Settings ******/
#define CLICK_TIME          300UL    // Relay ON time per click (ms). Short, because the motor moves while ON.
#define LIMIT_DEBOUNCE      20UL     // Same as the live firmware (ms)
#define BENCH_MOVE_TIMEOUT  60000UL  // Abort a leg after this (ms). Generous, because travel time is unknown.
#define LIMIT_CLEAR_TIME    1500UL   // Ship must leave the limit behind it within this time (ms)
#define REVERSE_PAUSE       2000UL   // Motor off between legs (ms), so it stops before reversing
#define ROUND_TRIPS         5

/****** Types ******/
struct LimitSwitch {
  uint8_t pin;
  bool active;              // Debounced: true = ship is at this limit
  bool lastRaw;             // Last raw reading, as "active"
  unsigned long lastChange; // When the raw reading last changed
};

struct BenchShip {
  uint8_t id;
  uint8_t relayToPos1;
  uint8_t relayToPos2;
  LimitSwitch limitPos1;
  LimitSwitch limitPos2;
};

enum Mode { IDLE, CLICKING, WATCHING, HOMING, MOVING, PAUSING };
enum Dir  { TO_POS1, TO_POS2 };

/****** Objects ******/
BenchShip ships[2] = {
  { 1, SHIP1_RELAY_TO_POS1, SHIP1_RELAY_TO_POS2, { SHIP1_LIMIT_POS1 }, { SHIP1_LIMIT_POS2 } },
  { 2, SHIP2_RELAY_TO_POS1, SHIP2_RELAY_TO_POS2, { SHIP2_LIMIT_POS1 }, { SHIP2_LIMIT_POS2 } }
};

Mode mode = IDLE;
BenchShip *runShip = nullptr;            // Ship being clicked or timed
Dir runDir = TO_POS1;
unsigned long modeStart = 0;
uint8_t legIndex = 0;                    // Even = to POS2, odd = to POS1
unsigned long legTimes[2][ROUND_TRIPS];  // [Dir][trip], ms
uint8_t watchRaw[4];                     // Last printed raw levels in watch mode

/****** Limit Switch Helpers ******/
void limitBegin(LimitSwitch &sw) {
  pinMode(sw.pin, INPUT_PULLUP);
  sw.lastRaw    = (digitalRead(sw.pin) == LIMIT_ACTIVE);
  sw.active     = sw.lastRaw;
  sw.lastChange = millis();
}

// Same debounce as the live firmware, so timings match what it will see.
void limitUpdate(LimitSwitch &sw) {
  bool raw = (digitalRead(sw.pin) == LIMIT_ACTIVE);
  if (raw != sw.lastRaw) {
    sw.lastRaw    = raw;
    sw.lastChange = millis();
  } else if (millis() - sw.lastChange >= LIMIT_DEBOUNCE) {
    sw.active = raw;
  }
}

LimitSwitch &aheadOf(BenchShip &s, Dir d)  { return (d == TO_POS1) ? s.limitPos1 : s.limitPos2; }
LimitSwitch &behindOf(BenchShip &s, Dir d) { return (d == TO_POS1) ? s.limitPos2 : s.limitPos1; }

/****** Motor Control — same rules as the live firmware ******/
void motorStop(BenchShip &s) {
  digitalWrite(s.relayToPos1, RELAY_OFF);
  digitalWrite(s.relayToPos2, RELAY_OFF);
}

void motorToPos1(BenchShip &s) {
  digitalWrite(s.relayToPos2, RELAY_OFF);
  digitalWrite(s.relayToPos1, RELAY_ON);
}

void motorToPos2(BenchShip &s) {
  digitalWrite(s.relayToPos1, RELAY_OFF);
  digitalWrite(s.relayToPos2, RELAY_ON);
}

void motorDrive(BenchShip &s, Dir d) {
  if (d == TO_POS1) motorToPos1(s); else motorToPos2(s);
}

void stopAll() {
  motorStop(ships[0]);
  motorStop(ships[1]);
}

/****** Output Helpers ******/
void printShipDir(const BenchShip &s, Dir d) {
  Serial.print(F("Ship "));
  Serial.print(s.id);
  Serial.print((d == TO_POS1) ? F(" to POS1") : F(" to POS2"));
}

void printMenu() {
  Serial.print(F("\n--- Space Ships bench test ---\n"));
  Serial.print(F("RELAY_ON = "));
  Serial.print(RELAY_ON == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F(", LIMIT_ACTIVE = "));
  Serial.print(LIMIT_ACTIVE == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F("\n1  click Ship 1 relay to POS1\n"));
  Serial.print(F("2  click Ship 1 relay to POS2\n"));
  Serial.print(F("3  click Ship 2 relay to POS1\n"));
  Serial.print(F("4  click Ship 2 relay to POS2\n"));
  Serial.print(F("w  watch raw limit levels\n"));
  Serial.print(F("a  time Ship 1 ("));
  Serial.print(ROUND_TRIPS);
  Serial.print(F(" round trips)\n"));
  Serial.print(F("b  time Ship 2 ("));
  Serial.print(ROUND_TRIPS);
  Serial.print(F(" round trips)\n"));
  Serial.print(F("s  stop all\n"));
  Serial.print(F("h  this menu\n"));
  Serial.print(F("Any key stops a running test.\n> "));
}

// Every failure ends here: motors off, reason printed, back to the menu.
void abortRun(const __FlashStringHelper *reason) {
  stopAll();
  mode = IDLE;
  Serial.print(F("\nABORT: "));
  Serial.print(reason);
  Serial.print(F("\n> "));
}

void finishRun() {
  stopAll();
  mode = IDLE;
  Serial.print(F("> "));
}

/****** Relay Click ******/
void startClick(BenchShip &s, Dir d) {
  if (s.limitPos1.active && s.limitPos2.active) {
    abortRun(F("both limits active — check wiring"));
    return;
  }
  if (aheadOf(s, d).active) {
    printShipDir(s, d);
    Serial.print(F(": already at that limit, not driving into the end\n> "));
    return;
  }
  runShip   = &s;
  runDir    = d;
  modeStart = millis();
  mode      = CLICKING;
  printShipDir(s, d);
  Serial.print(F(": relay ON\n"));
  motorDrive(s, d);
}

void updateClick() {
  LimitSwitch &ahead = aheadOf(*runShip, runDir);
  if (ahead.active) {
    Serial.print(F("Reached limit, relay OFF\n"));
    finishRun();
  } else if (millis() - modeStart >= CLICK_TIME) {
    Serial.print(F("Relay OFF\n"));
    finishRun();
  }
}

/****** Limit Watch ******/
const uint8_t WATCH_PINS[4] = { SHIP1_LIMIT_POS1, SHIP1_LIMIT_POS2, SHIP2_LIMIT_POS1, SHIP2_LIMIT_POS2 };

void printWatchLine() {
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(F("S"));
    Serial.print(i / 2 + 1);
    Serial.print((i % 2) ? F("P2") : F("P1"));
    Serial.print(watchRaw[i] == HIGH ? F("=HIGH") : F("=LOW "));
    Serial.print(watchRaw[i] == LIMIT_ACTIVE ? F("* ") : F("  "));
  }
  Serial.print(F("\n"));
}

void readWatchPins() {
  for (uint8_t i = 0; i < 4; i++) watchRaw[i] = digitalRead(WATCH_PINS[i]);
}

void startWatch() {
  mode = WATCHING;
  Serial.print(F("Raw pin levels, printed on change. * = active with current LIMIT_ACTIVE.\n"));
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

/****** Travel Timing ******/
void beginLeg() {
  runDir    = (legIndex % 2 == 0) ? TO_POS2 : TO_POS1;
  modeStart = millis();
  mode      = MOVING;
  motorDrive(*runShip, runDir);
}

void startTiming(BenchShip &s) {
  runShip = &s;
  if (s.limitPos1.active && s.limitPos2.active) {
    abortRun(F("both limits active — check wiring"));
    return;
  }
  Serial.print(F("Ship "));
  Serial.print(s.id);
  Serial.print(F(": timing "));
  Serial.print(ROUND_TRIPS);
  Serial.print(F(" round trips. Any key aborts.\n"));
  legIndex  = 0;
  modeStart = millis();
  if (s.limitPos1.active) {
    mode = PAUSING;  // Already home
  } else {
    Serial.print(F("Homing to POS1 (untimed)\n"));
    runDir = TO_POS1;
    mode   = HOMING;
    motorDrive(s, TO_POS1);
  }
}

// Time from motor ON to the first raw edge of the ahead switch, not the debounced edge.
unsigned long arrivalTime(const LimitSwitch &ahead) {
  long t = (long)(ahead.lastChange - modeStart);
  return (t > 0) ? (unsigned long)t : millis() - modeStart;
}

// Same checks as the live firmware. Returns true on arrival; aborts on any fault.
bool moveArrived() {
  LimitSwitch &ahead  = aheadOf(*runShip, runDir);
  LimitSwitch &behind = behindOf(*runShip, runDir);
  unsigned long elapsed = millis() - modeStart;

  if (ahead.active && behind.active) {
    abortRun(F("both limits active — check wiring"));
  } else if (ahead.active) {
    motorStop(*runShip);
    return true;
  } else if (behind.active && elapsed >= LIMIT_CLEAR_TIME) {
    abortRun(F("limit behind ship still active — motor reversed or ship jammed"));
  } else if (elapsed >= BENCH_MOVE_TIMEOUT) {
    abortRun(F("move timeout — limit never reached"));
  }
  return false;
}

void printDirStats(Dir d) {
  unsigned long sum = 0, lo = ULONG_MAX, hi = 0;
  for (uint8_t i = 0; i < ROUND_TRIPS; i++) {
    unsigned long t = legTimes[d][i];
    sum += t;
    if (t < lo) lo = t;
    if (t > hi) hi = t;
  }
  Serial.print((d == TO_POS1) ? F("| to POS1 | ") : F("| to POS2 | "));
  Serial.print(sum / ROUND_TRIPS);
  Serial.print(F(" | "));
  Serial.print(lo);
  Serial.print(F(" | "));
  Serial.print(hi);
  Serial.print(F(" |\n"));
}

unsigned long dirAverage(Dir d) {
  unsigned long sum = 0;
  for (uint8_t i = 0; i < ROUND_TRIPS; i++) sum += legTimes[d][i];
  return sum / ROUND_TRIPS;
}

void printResults() {
  Serial.print(F("\nShip "));
  Serial.print(runShip->id);
  Serial.print(F(" travel times, ms\n"));
  Serial.print(F("| Direction | Avg | Min | Max |\n"));
  Serial.print(F("|---|---|---|---|\n"));
  printDirStats(TO_POS1);
  printDirStats(TO_POS2);
  unsigned long slowest = max(dirAverage(TO_POS1), dirAverage(TO_POS2));
  Serial.print(F("1.5 x slowest avg = "));
  Serial.print(slowest * 3 / 2);
  Serial.print(F(" ms\n"));
}

void onLegArrived() {
  unsigned long t = arrivalTime(aheadOf(*runShip, runDir));
  legTimes[runDir][legIndex / 2] = t;
  Serial.print(F("Leg "));
  Serial.print(legIndex + 1);
  Serial.print(F("/"));
  Serial.print(2 * ROUND_TRIPS);
  Serial.print(F(": "));
  printShipDir(*runShip, runDir);
  Serial.print(F(" "));
  Serial.print(t);
  Serial.print(F(" ms\n"));

  legIndex++;
  if (legIndex >= 2 * ROUND_TRIPS) {
    printResults();
    finishRun();
  } else {
    mode      = PAUSING;
    modeStart = millis();
  }
}

void updateTiming() {
  switch (mode) {
    case HOMING:
      if (moveArrived()) {
        Serial.print(F("At POS1\n"));
        mode      = PAUSING;
        modeStart = millis();
      }
      break;
    case MOVING:
      if (moveArrived()) onLegArrived();
      break;
    case PAUSING:
      if (millis() - modeStart >= REVERSE_PAUSE) beginLeg();
      break;
    default:
      break;
  }
}

/****** Serial Commands ******/
void handleIdleKey(char c) {
  switch (c) {
    case '1': startClick(ships[0], TO_POS1); break;
    case '2': startClick(ships[0], TO_POS2); break;
    case '3': startClick(ships[1], TO_POS1); break;
    case '4': startClick(ships[1], TO_POS2); break;
    case 'w': startWatch(); break;
    case 'a': startTiming(ships[0]); break;
    case 'b': startTiming(ships[1]); break;
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

  if (mode == WATCHING) {
    Serial.print(F("Watch ended\n> "));
    mode = IDLE;
  } else if (mode != IDLE) {
    abortRun(F("stopped by key"));
  } else {
    handleIdleKey(c);
  }
}

/****** Status LED ******/
// ON while a motor is driven, so a running motor is visible without the monitor.
void updateStatusLED() {
  bool driving = (mode == CLICKING || mode == HOMING || mode == MOVING);
  digitalWrite(STATUS_LED, driving ? HIGH : LOW);
}

/****** Setup ******/
void setup() {
  // Relays first, so motors are held OFF as early as possible.
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(ships[i].relayToPos1, OUTPUT);
    pinMode(ships[i].relayToPos2, OUTPUT);
    motorStop(ships[i]);
  }
  for (uint8_t i = 0; i < 2; i++) {
    limitBegin(ships[i].limitPos1);
    limitBegin(ships[i].limitPos2);
  }
  pinMode(STATUS_LED, OUTPUT);

  Serial.begin(115200);
  printMenu();

  wdt_enable(WDT_TIMEOUT);
}

/****** Main Loop ******/
void loop() {
  wdt_reset();
  for (uint8_t i = 0; i < 2; i++) {
    limitUpdate(ships[i].limitPos1);
    limitUpdate(ships[i].limitPos2);
  }
  handleSerial();

  switch (mode) {
    case CLICKING: updateClick(); break;
    case WATCHING: updateWatch(); break;
    case HOMING:
    case MOVING:
    case PAUSING:  updateTiming(); break;
    case IDLE:     break;
  }
  updateStatusLED();
}
