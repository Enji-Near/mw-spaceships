#include <Arduino.h>
#include <avr/wdt.h>

/*****  Space Ships — v0.2  *****
 *  Platform: Arduino Nano Every (ATmega4809, 5V logic)
 *
 *  Two spaceships shuttle back and forth on rope-and-pulley "clotheslines".
 *  Per ship: one 12V DC motor, 2 relays (polarity reversal), 2 limit switches.
 *  Cycle (repeats while powered, each ship independent):
 *    move to POS2 → hit limit → stop → dwell → move to POS1 → hit limit → stop → dwell → ...
 *
 *  Relay wiring — 2x SPDT relays per motor ("two-relay H-bridge"):
 *    Motor terminal A → COM of relay "TO_POS1"; motor terminal B → COM of relay "TO_POS2"
 *    Each relay: NC → GND (12V return), NO → +12V
 *    Both OFF = both motor terminals on GND = stopped AND braked
 *    One ON   = motor driven in that direction
 *    Both ON  = both terminals on +12V = also stopped (no supply short in this wiring),
 *               but the code still never does it.
 *
 *  Relay modules: 12V optocoupler modules, trigger jumper set to H (high-level trigger).
 *    WHY H: in L mode on a 12V module, a 5V "HIGH" from the Arduino may not fully
 *    turn the input off (the module's input side sits at 12V). H mode avoids that,
 *    and a floating pin during reset/upload leaves the relay OFF. Verify on the bench.
 *
 *  Limit switches — wired pin → switch → GND, INPUT_PULLUP.
 *    Currently assumed NO (normally open): at limit = switch closed = LOW.
 *    If the switches have an NC terminal, prefer NC and set LIMIT_ACTIVE to HIGH:
 *    a broken wire then reads as "at limit" and stops the motor (fail-safe).
 *    With NO, a broken wire reads as "not at limit"; the move timeout is the backstop.
 *
 *  Safety layers:
 *    1. Move timeout — stop if the target limit is never reached (slipped rope, jam, dead switch)
 *    2. Wrong-way check — limit BEHIND the ship still active after LIMIT_CLEAR_TIME
 *       = motor wired backwards or ship jammed
 *    3. Both-limits check — mechanically impossible → wiring fault
 *    4. Fault → cool-down → retry; lockout after MAX_FAULTS in a row (power cycle clears)
 *    5. Hardware watchdog (~2 s) — any software hang resets to a safe boot state
 *
 *  Status LED (D13, onboard): slow blink = running, fast blink = fault on either ship.
 *  Avoid D0/D1 — hardware USART (Serial1).
 *********************************/

#define DEBUG 1  // 1 = serial debug output (115200 baud), 0 = deployment

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

/****** Timing Constants ******/
#define LIMIT_DEBOUNCE     20UL     // Limit must read steady this long (ms).
                                    // Longer = more noise immunity, but more overtravel past the switch.
#define DWELL_TIME         5000UL   // Pause at each end (ms)
#define MOVE_TIMEOUT       20000UL  // Max end-to-end travel (ms) — high estimate; set to ~1.5x measured travel time
#define LIMIT_CLEAR_TIME   1500UL   // Ship must leave the limit behind it within this time (ms)
#define FAULT_RETRY_TIME   10000UL  // Cool-down before retrying after a fault (ms).
                                    // Lets a stalled motor cool and a snag clear. MAX_FAULTS bounds total stall time.
#define MAX_FAULTS         3        // Consecutive faults before lockout
#define BLINK_NORMAL       1000UL   // Status LED toggle interval, running (ms)
#define BLINK_FAULT        150UL    // Status LED toggle interval, fault (ms)

// Watchdog constant differs between chip families.
#ifdef ARDUINO_ARCH_MEGAAVR
  #define WDT_TIMEOUT  WDT_PERIOD_2KCLK_gc  // ATmega4809: ~2 s
#else
  #define WDT_TIMEOUT  WDTO_2S              // Classic AVR (Uno/Nano)
#endif

/****** Types ******/
enum ShipState {
  MOVING_TO_POS1,
  DWELL_AT_POS1,
  MOVING_TO_POS2,
  DWELL_AT_POS2,
  FAULT,        // Motor off, waiting FAULT_RETRY_TIME, then retry
  LOCKED_OUT    // Motor off until power cycle — too many faults in a row
};

// One debounced limit switch
struct LimitSwitch {
  uint8_t pin;
  bool active;              // Debounced: true = ship is at this limit
  bool lastRaw;             // Last raw reading
  unsigned long lastChange; // When the raw reading last changed
};

// Everything that belongs to one ship
struct Ship {
  uint8_t id;
  uint8_t relayToPos1;
  uint8_t relayToPos2;
  LimitSwitch limitPos1;
  LimitSwitch limitPos2;
  ShipState state;
  ShipState lastMove;       // Last direction of travel — used to choose retry direction
  unsigned long stateStart; // When the current state began
  uint8_t faultCount;       // Consecutive faults — reset on every successful arrival
};

/****** Objects ******/
Ship ship1 = { 1, SHIP1_RELAY_TO_POS1, SHIP1_RELAY_TO_POS2, { SHIP1_LIMIT_POS1 }, { SHIP1_LIMIT_POS2 } };
Ship ship2 = { 2, SHIP2_RELAY_TO_POS1, SHIP2_RELAY_TO_POS2, { SHIP2_LIMIT_POS1 }, { SHIP2_LIMIT_POS2 } };

/****** Limit Switch Helpers ******/
void limitBegin(LimitSwitch &sw) {
  pinMode(sw.pin, INPUT_PULLUP);
  sw.lastRaw    = (digitalRead(sw.pin) == LIMIT_ACTIVE);
  sw.active     = sw.lastRaw;   // Trust first reading at boot
  sw.lastChange = millis();
}

// Accept a new level only after it has been steady for LIMIT_DEBOUNCE ms.
// Rejects contact bounce and short noise spikes picked up by long wires.
void limitUpdate(LimitSwitch &sw) {
  bool raw = (digitalRead(sw.pin) == LIMIT_ACTIVE);
  if (raw != sw.lastRaw) {
    sw.lastRaw    = raw;
    sw.lastChange = millis();
  } else if (millis() - sw.lastChange >= LIMIT_DEBOUNCE) {
    sw.active = raw;
  }
}

/****** Motor Control — 2-Relay Polarity Reversal ******/
// Each function turns the OPPOSITE relay off before turning its own on.
// The state machine only starts a move from a stopped state, so the dwell
// (or fault cool-down) is the dead time between directions — no delay() needed.
void motorStop(Ship &s) {
  digitalWrite(s.relayToPos1, RELAY_OFF);
  digitalWrite(s.relayToPos2, RELAY_OFF);
}

void motorToPos1(Ship &s) {
  digitalWrite(s.relayToPos2, RELAY_OFF);
  digitalWrite(s.relayToPos1, RELAY_ON);
}

void motorToPos2(Ship &s) {
  digitalWrite(s.relayToPos1, RELAY_OFF);
  digitalWrite(s.relayToPos2, RELAY_ON);
}

/****** Debug Output ******/
void debugState(const Ship &s) {
  #if DEBUG
  Serial.print(F("Ship "));
  Serial.print(s.id);
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
// Single entry point for every state change: sets state, timestamps it,
// and applies the motor output for that state. Keeps outputs and state in sync.
void enterState(Ship &s, ShipState next) {
  s.state      = next;
  s.stateStart = millis();

  switch (next) {
    case MOVING_TO_POS1: s.lastMove = next; motorToPos1(s); break;
    case MOVING_TO_POS2: s.lastMove = next; motorToPos2(s); break;
    default:             motorStop(s); break;  // Dwell, fault, lockout: motor OFF
  }
  debugState(s);
}

void shipFault(Ship &s, const __FlashStringHelper *reason) {
  s.faultCount++;
  #if DEBUG
  Serial.print(F("Ship "));
  Serial.print(s.id);
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

// Retry after cool-down. If the ship is at a limit, head for the other end.
// If it stopped mid-track, go back the way it came — that path last worked.
void shipRecover(Ship &s) {
  if (s.limitPos1.active && s.limitPos2.active) {
    shipFault(s, F("both limits active — check wiring"));
  } else if (s.limitPos1.active) {
    enterState(s, MOVING_TO_POS2);
  } else if (s.limitPos2.active) {
    enterState(s, MOVING_TO_POS1);
  } else {
    enterState(s, (s.lastMove == MOVING_TO_POS1) ? MOVING_TO_POS2 : MOVING_TO_POS1);
  }
}

// Shared logic for both directions of travel.
//   ahead  = the limit the ship is driving toward
//   behind = the limit the ship is driving away from
void updateMove(Ship &s, LimitSwitch &ahead, LimitSwitch &behind, ShipState arrivedState) {
  unsigned long elapsed = millis() - s.stateStart;

  if (ahead.active) {
    s.faultCount = 0;              // Successful arrival clears the fault history
    enterState(s, arrivedState);
  }
  else if (behind.active && elapsed >= LIMIT_CLEAR_TIME) {
    shipFault(s, F("limit behind ship still active — motor reversed or ship jammed"));
  }
  else if (elapsed >= MOVE_TIMEOUT) {
    shipFault(s, F("move timeout — limit never reached"));
  }
}

void updateShip(Ship &s) {
  limitUpdate(s.limitPos1);
  limitUpdate(s.limitPos2);

  // Both limits active is mechanically impossible → wiring or switch fault.
  // Skipped in FAULT/LOCKED_OUT: the motor is already off there.
  if (s.state != FAULT && s.state != LOCKED_OUT &&
      s.limitPos1.active && s.limitPos2.active) {
    shipFault(s, F("both limits active — check wiring"));
    return;
  }

  unsigned long elapsed = millis() - s.stateStart;

  switch (s.state) {
    case MOVING_TO_POS1: updateMove(s, s.limitPos1, s.limitPos2, DWELL_AT_POS1); break;
    case MOVING_TO_POS2: updateMove(s, s.limitPos2, s.limitPos1, DWELL_AT_POS2); break;
    case DWELL_AT_POS1:  if (elapsed >= DWELL_TIME) enterState(s, MOVING_TO_POS2); break;
    case DWELL_AT_POS2:  if (elapsed >= DWELL_TIME) enterState(s, MOVING_TO_POS1); break;
    case FAULT:          if (elapsed >= FAULT_RETRY_TIME) shipRecover(s); break;
    case LOCKED_OUT:     break;  // Stay stopped until power cycle
  }
}

/****** Ship Startup ******/
void shipBegin(Ship &s) {
  // Relays first, so motors are held OFF as early as possible.
  // megaAVR core: pinMode(OUTPUT) drives the pin at its OUT register value (LOW)
  // until the next digitalWrite. With RELAY_OFF = LOW that is already the safe state.
  pinMode(s.relayToPos1, OUTPUT);
  pinMode(s.relayToPos2, OUTPUT);
  motorStop(s);

  limitBegin(s.limitPos1);
  limitBegin(s.limitPos2);
  s.faultCount = 0;

  // Position is unknown after power-up — work it out from the limits.
  if (s.limitPos1.active && s.limitPos2.active) {
    shipFault(s, F("both limits active at boot — check wiring"));
  } else if (s.limitPos1.active) {
    enterState(s, DWELL_AT_POS1);
  } else if (s.limitPos2.active) {
    enterState(s, DWELL_AT_POS2);
  } else {
    enterState(s, MOVING_TO_POS1);  // Mid-track: home to position 1
  }
}

/****** Status LED ******/
bool isFaulted(const Ship &s) {
  return (s.state == FAULT || s.state == LOCKED_OUT);
}

void updateStatusLED() {
  static unsigned long lastToggle = 0;
  static bool ledOn = false;

  unsigned long interval = (isFaulted(ship1) || isFaulted(ship2)) ? BLINK_FAULT : BLINK_NORMAL;
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
  Serial.print(F("Space Ships v0.2: Online\n"));
  #endif

  pinMode(STATUS_LED, OUTPUT);

  shipBegin(ship1);
  shipBegin(ship2);

  // Watchdog — enabled last, after all setup work is done.
  // Loop is fully non-blocking, so wdt_reset() runs thousands of times per second.
  wdt_enable(WDT_TIMEOUT);
}

/****** Main Loop ******/
void loop() {
  wdt_reset();
  updateShip(ship1);
  updateShip(ship2);
  updateStatusLED();
}
