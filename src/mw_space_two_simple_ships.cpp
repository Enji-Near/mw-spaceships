#include <Arduino.h>
#include <avr/wdt.h>
#include "ship_motion.h"

/*****  Space Ships — v0.4 live firmware  *****
 *  Platform: Arduino Nano Every (ATmega4809, 5V logic). One DRV8871 per ship.
 *  Cycle per ship (independent): move → hit limit → brake → DWELL_TIME → reverse, repeating.
 *  Speed per ship from ship_config.h: MODE_STEADY or MODE_SURGE. Fixed values, nothing adapts.
 *  All tunable values: include/ship_config.h. Motor and limit code: ship_motion.cpp (shared).
 *
 *  Safety layers:
 *    1. Move timeout — target limit never reached (slipped rope, jam, dead switch)
 *    2. Both-limits check — mechanically impossible → wiring fault
 *    3. Fault → brake, cool-down, retry; lockout after MAX_FAULTS in a row (power cycle clears)
 *    4. Hardware watchdog (~2 s). Reset = driver inputs low = coast.
 *
 *  Imperial March (MARCH_ON_POWER_UP, MARCH_ON_RESET, MARCH_ON_BUTTON on A4, MARCH_ON_SERIAL): Ship 1 homes the way it was going,
 *  Ship 2 homes to POS2, both sing away from their limit, then the normal cycle carries on.
 *  Same safety checks; fault retries wait for the song; faulted ships sit it out.
 *********************************************/

/****** Types ******/
enum ShipState {
  MOVING_TO_POS1,
  DWELL_AT_POS1,
  MOVING_TO_POS2,
  DWELL_AT_POS2,
  FAULT,        // Braked, waiting FAULT_RETRY_TIME, then retry
  LOCKED_OUT,   // Braked until power cycle
  MARCH_BRAKE,         // Button march: braked for MARCH_BRAKE_MS before homing
  MARCH_HOME_TO_POS1,  // March: profile speed to a limit
  MARCH_HOME_TO_POS2,
  MARCH_READY,         // At a limit, braked, waiting for the song
  MARCH_SING_TO_POS1,  // MARCH_DUTY_PCT, tone PWM
  MARCH_SING_TO_POS2,
  MARCH_DONE           // Reached a limit during the song, braked until it ends
};

enum MarchPhase { M_OFF, M_BRAKE, M_HOMING, M_DELAY, M_SONG };

MarchPhase marchPhase = M_OFF;
unsigned long marchPhaseStart = 0;

// Direction of travel, kept through a reset (not power-up): .noinit is not cleared at startup.
struct DirMemory { uint8_t magic; uint8_t dir[2]; };
DirMemory dirMem __attribute__((section(".noinit")));
#define DIR_MAGIC  0x5A

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
    case MARCH_BRAKE:        Serial.print(F(": march, braking\n")); break;
    case MARCH_HOME_TO_POS1: Serial.print(F(": march, homing to position 1\n")); break;
    case MARCH_HOME_TO_POS2: Serial.print(F(": march, homing to position 2\n")); break;
    case MARCH_READY:        Serial.print(F(": march, ready\n")); break;
    case MARCH_SING_TO_POS1: Serial.print(F(": march, singing to position 1\n")); break;
    case MARCH_SING_TO_POS2: Serial.print(F(": march, singing to position 2\n")); break;
    case MARCH_DONE:         Serial.print(F(": march, reached a limit, waiting for the song to end\n")); break;
  }
  #endif
}

/****** State Machine ******/
bool isMove(ShipState st) {
  return st == MOVING_TO_POS1 || st == MOVING_TO_POS2 || st == MARCH_HOME_TO_POS1 || st == MARCH_HOME_TO_POS2;
}
bool isSing(ShipState st) { return st == MARCH_SING_TO_POS1 || st == MARCH_SING_TO_POS2; }
Dir moveDir(ShipState st) {
  return (st == MOVING_TO_POS1 || st == MARCH_HOME_TO_POS1 || st == MARCH_SING_TO_POS1) ? TO_POS1 : TO_POS2;
}

void rememberDir(const Ship &s, Dir d) {
  dirMem.dir[s.io.id - 1] = d;
  dirMem.magic = DIR_MAGIC;
}

Dir recalledDir(const Ship &s) {
  uint8_t d = dirMem.dir[s.io.id - 1];
  return (dirMem.magic == DIR_MAGIC && d <= TO_POS2) ? (Dir)d : TO_POS1;  // Unknown: POS1, as at power-up
}

// Single entry point for every state change, so outputs and state stay in sync.
void enterState(Ship &s, ShipState next) {
  s.state      = next;
  s.stateStart = millis();
  if (isMove(next) || isSing(next)) {
    Dir d = moveDir(next);
    s.lastMove = (d == TO_POS1) ? MOVING_TO_POS1 : MOVING_TO_POS2;
    rememberDir(s, d);
    if (isSing(next)) motorDriveTone(s.io, d, PCT_TO_DUTY(MARCH_DUTY_PCT));
    else              motorDrive(s.io, d, tripDuty(s.tune, 0));
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
  } else if (elapsed >= MOVE_TIMEOUT) {
    shipFault(s, F("move timeout — limit never reached"));
  } else {
    motorDrive(s.io, d, tripDuty(s.tune, elapsed));  // Only rewrites PWM when duty changes
  }
}

// Song move: fixed duty with tone, until the limit or the end of the song.
void updateSing(Ship &s) {
  Dir d = moveDir(s.state);
  if (aheadOf(s.io, d).active) {
    s.faultCount = 0;
    enterState(s, MARCH_DONE);
  } else {
    motorDriveTone(s.io, d, PCT_TO_DUTY(MARCH_DUTY_PCT));
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
    case FAULT:          if (elapsed >= FAULT_RETRY_TIME && marchPhase == M_OFF) shipRecover(s); break;  // Not over the song's PWM
    case LOCKED_OUT:     break;
    case MARCH_HOME_TO_POS1:
    case MARCH_HOME_TO_POS2: updateMove(s, MARCH_READY); break;
    case MARCH_SING_TO_POS1:
    case MARCH_SING_TO_POS2: updateSing(s); break;
    case MARCH_BRAKE:
    case MARCH_READY:
    case MARCH_DONE:         break;
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

/****** Imperial March (power-up, button) ******/
// Ship 2 always starts at POS2: toward POS1 its PWM pin (D5) does drive/brake like Ship 1, so every
// song sounds the same. Ship 1 sounds the same both ways: mid-track it carries on the way it was going.
bool marchAtStart(const Ship &s) { return (s.io.id == 2) ? s.io.pos2.active : (s.io.pos1.active || s.io.pos2.active); }
Dir marchSingDir(const Ship &s)  { return (s.io.id == 2 || !s.io.pos1.active) ? TO_POS1 : TO_POS2; }

void marchShipEnter(Ship &s) {
  if (marchAtStart(s))   enterState(s, MARCH_READY);
  else if (s.io.id == 2) enterState(s, MARCH_HOME_TO_POS2);
  else enterState(s, (recalledDir(s) == TO_POS1) ? MARCH_HOME_TO_POS1 : MARCH_HOME_TO_POS2);
}

void marchShipStart(Ship &s) {  // At boot
  shipBegin(s.io);
  s.faultCount = 0;
  if (bothLimits(s.io)) shipFault(s, F("both limits active at boot — check wiring"));
  else                  marchShipEnter(s);
}

LimitSwitch marchButton = { MARCH_BUTTON_PIN, LOW, false, false, 0 };

void buttonBegin() {
  pinMode(MARCH_BUTTON_PIN, INPUT_PULLUP);
  marchButton.lastRaw    = (digitalRead(MARCH_BUTTON_PIN) == LOW);
  marchButton.active     = marchButton.lastRaw;  // Held at boot: needs a release first
  marchButton.lastChange = millis();
}

// True once per press (debounced edge).
bool buttonPressed() {
  bool was = marchButton.active;
  limitUpdate(marchButton);
  return marchButton.active && !was;
}

// Button: every ship that isn't faulted brakes, then homes. Faulted ships sit the song out.
void marchButtonStart() {
  bool any = false;
  for (uint8_t i = 0; i < 2; i++) {
    Ship &s = ships[i];
    if (s.state == FAULT || s.state == LOCKED_OUT) continue;
    enterState(s, MARCH_BRAKE);
    any = true;
  }
  if (!any) return;
  marchPhase      = M_BRAKE;
  marchPhaseStart = millis();
}

bool marchHoming(const Ship &s) { return s.state == MARCH_HOME_TO_POS1 || s.state == MARCH_HOME_TO_POS2; }

// Song over: tone off, then each ship back into the normal cycle.
void marchEnd() {
  for (uint8_t i = 0; i < 2; i++) if (isSing(ships[i].state)) motorBrake(ships[i].io);
  toneEnd();
  marchPhase = M_OFF;
  for (uint8_t i = 0; i < 2; i++) {
    Ship &s = ships[i];
    if (isSing(s.state)) {
      enterState(s, (moveDir(s.state) == TO_POS1) ? MOVING_TO_POS1 : MOVING_TO_POS2);  // Carry on to that end
    } else if (s.state == MARCH_DONE || s.state == MARCH_READY) {
      enterState(s, s.io.pos1.active ? DWELL_AT_POS1 : DWELL_AT_POS2);
    }
  }
  #if DEBUG
  Serial.print(F("March over, normal cycle\n"));
  #endif
}

// True if the command byte arrived. Reads every byte, so input during the song is dropped.
bool serialMarchCmd() {
  bool cmd = false;
  while (Serial.available() > 0) if (Serial.read() == MARCH_SERIAL_CMD) cmd = true;
  return cmd;
}

void updateMarch() {
  bool pressed = buttonPressed();  // Every loop, so presses during the song are seen and dropped
  bool serialCmd = serialMarchCmd();
  switch (marchPhase) {
    case M_OFF:
      if ((MARCH_ON_BUTTON && pressed) || (MARCH_ON_SERIAL && serialCmd)) marchButtonStart();
      return;
    case M_BRAKE:
      if (millis() - marchPhaseStart < MARCH_BRAKE_MS) return;
      for (uint8_t i = 0; i < 2; i++) if (ships[i].state == MARCH_BRAKE) marchShipEnter(ships[i]);
      marchPhase = M_HOMING;
      return;
    case M_HOMING:
      if (marchHoming(ships[0]) || marchHoming(ships[1])) return;
      if (ships[0].state != MARCH_READY && ships[1].state != MARCH_READY) { marchEnd(); return; }  // Both faulted
      marchPhase      = M_DELAY;
      marchPhaseStart = millis();
      return;
    case M_DELAY:
      if (millis() - marchPhaseStart < MARCH_START_DELAY_MS) return;
      for (uint8_t i = 0; i < 2; i++) {
        Ship &s = ships[i];
        if (s.state == MARCH_READY) enterState(s, (marchSingDir(s) == TO_POS1) ? MARCH_SING_TO_POS1 : MARCH_SING_TO_POS2);
      }
      marchSongStart();
      marchPhase = M_SONG;
      return;
    case M_SONG:
      if (marchSongUpdate() && (isSing(ships[0].state) || isSing(ships[1].state))) return;
      marchEnd();
      return;
  }
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
  uint8_t rf = RSTCTRL.RSTFR;  // Why we booted (ATmega4809 RSTCTRL)
  RSTCTRL.RSTFR = rf;          // Write 1s to clear, so the next boot reads only its own cause
  bool powerUp  = rf & (RSTCTRL_PORF_bm | RSTCTRL_BORF_bm);
  bool resetPin = (rf & RSTCTRL_EXTRF_bm) && !powerUp;  // Reset button or USB connect
  bool march    = (MARCH_ON_POWER_UP && powerUp) || (MARCH_ON_RESET && resetPin);  // Never after watchdog or upload
  if (powerUp) dirMem.magic = 0;  // .noinit is random after power-up

  Serial.begin(115200);  // Always on: the serial march command needs RX
  #if DEBUG
  Serial.print(F("Space Ships v0.4: Online. Reset flags 0x"));
  Serial.print(rf, HEX);
  Serial.print(powerUp ? F(" (power-up)\n") : resetPin ? F(" (reset pin)\n") : F("\n"));
  #endif
  pinMode(STATUS_LED, OUTPUT);
  buttonBegin();
  if (march) {
    #if DEBUG
    Serial.print(F("Imperial March\n"));
    #endif
    marchShipStart(ships[0]);
    marchShipStart(ships[1]);
    marchPhase = M_HOMING;
  } else {
    shipStart(ships[0]);
    shipStart(ships[1]);
  }
  wdt_enable(WDT_TIMEOUT);  // Last, after setup. Loop is non-blocking.
}

/****** Main Loop ******/
void loop() {
  wdt_reset();
  updateShip(ships[0]);
  updateShip(ships[1]);
  updateMarch();
  updateStatusLED();
}
