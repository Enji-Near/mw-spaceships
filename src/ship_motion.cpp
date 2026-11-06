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

/****** Tone mode ******/
// TCA0 single-slope: output HIGH (= brake) while CNT < CMP. Normal PER is 255 (core PWM_TIMER_PERIOD),
// so analogWrite writes 0-255 straight into CMP. In tone mode PER sets the pitch and CMP is rescaled.
#define TCA_HZ  (F_CPU / PWM_TCA_DIV)
static uint16_t tcaPer = 255;
static uint8_t tcaHigh[3] = { 0, 0, 0 };  // Per TCA0 channel: HIGH share 0-255

static void tcaWriteCmp(uint8_t ch) {
  volatile uint16_t *cmp = &TCA0.SINGLE.CMP0BUF + ch;  // CMP0BUF..CMP2BUF are consecutive (core does the same)
  *cmp = (uint16_t)((uint32_t)tcaHigh[ch] * (tcaPer + 1UL) / 256);
}

void toneSetHz(uint16_t hz) {
  uint32_t per = 255;
  if (hz > 0) {
    per = TCA_HZ / hz - 1;
    if (per > 0xFFFF) per = 0xFFFF;  // Lowest note TCA_HZ / 65536 (~122 Hz at /2)
    if (per < 255) per = 255;        // Never above the normal PWM frequency
  }
  uint8_t sreg = SREG;
  cli();  // 16-bit buffer writes
  tcaPer = (uint16_t)per;
  TCA0.SINGLE.PERBUF = tcaPer;      // Buffered: PER and CMP change together at the next overflow
  for (uint8_t ch = 0; ch < 3; ch++) tcaWriteCmp(ch);
  SREG = sreg;
}

static void tcaPwm(uint8_t pin, uint8_t high) {
  uint8_t ch = digitalPinToBitPosition(pin);  // TCA0 WOn is PORTB bit n (core variant)
  tcaHigh[ch] = high;
  uint8_t sreg = SREG;
  cli();
  tcaWriteCmp(ch);
  SREG = sreg;
  TCA0.SINGLE.CTRLB |= (1 << (TCA_SINGLE_CMP0EN_bp + ch));
}

// Drive/brake when the other pin is on TCA0 (same as motorDrive). If only the target pin is
// (Ship 2 toward POS2: other pin is D3 on TCB1), drive/coast instead: PWM the target pin, other LOW.
// Coast > ~1 ms puts the DRV8871 to sleep; it wakes 50 us after an input goes high (TI SLVSCY9B §7.3.2).
void motorDriveTone(ShipIO &s, Dir d, uint8_t duty) {
  uint8_t target = (d == TO_POS1) ? s.inToPos1 : s.inToPos2;
  uint8_t other  = (d == TO_POS1) ? s.inToPos2 : s.inToPos1;
  if (duty == 0 || duty == 255) {
    motorDrive(s, d, duty);
  } else if (digitalPinToTimer(other) == TIMERA0) {
    digitalWrite(target, HIGH);
    tcaPwm(other, 255 - duty);   // HIGH = brake
  } else if (digitalPinToTimer(target) == TIMERA0) {
    digitalWrite(other, LOW);    // Also stops D3's TCB PWM
    tcaPwm(target, duty);        // HIGH = drive, LOW = coast
  } else {
    motorDrive(s, d, duty);      // No TCA0 pin: silent
  }
  s.lastCmd = -1;  // So the next motorDrive rewrites both pins
}

void toneEnd() {
  for (uint8_t ch = 0; ch < 3; ch++) tcaHigh[ch] = 0;
  toneSetHz(0);
}

/****** Imperial March song (tone mode) ******/
// Common arrangement, A minor. hz 0 = rest. Lengths in ms at MARCH_TEMPO_PCT 100.
struct Note { uint16_t hz; uint16_t ms; };
#define N_F4   349
#define N_GS4  415
#define N_A4   440
#define N_AS4  466
#define N_B4   494
#define N_C5   523
#define N_CS5  554
#define N_D5   587
#define N_DS5  622
#define N_E5   659
#define N_F5   698
#define N_FS5  740
#define N_G5   784
#define N_GS5  831
#define N_A5   880
const Note MARCH[] PROGMEM = {
  // Theme, twice
  { N_A4, 500 }, { N_A4, 500 }, { N_A4, 500 }, { N_F4, 350 }, { N_C5, 150 },
  { N_A4, 500 }, { N_F4, 350 }, { N_C5, 150 }, { N_A4, 650 }, { 0, 500 },
  { N_E5, 500 }, { N_E5, 500 }, { N_E5, 500 }, { N_F5, 350 }, { N_C5, 150 },
  { N_GS4, 500 }, { N_F4, 350 }, { N_C5, 150 }, { N_A4, 650 }, { 0, 500 },
  // Second section
  { N_A5, 500 }, { N_A4, 300 }, { N_A4, 150 }, { N_A5, 500 }, { N_GS5, 325 }, { N_G5, 175 },
  { N_FS5, 125 }, { N_F5, 125 }, { N_FS5, 250 }, { 0, 325 },
  { N_AS4, 250 }, { N_DS5, 500 }, { N_D5, 325 }, { N_CS5, 175 }, { N_C5, 125 }, { N_B4, 125 },
  { N_C5, 250 }, { 0, 350 },
  // Variant 1
  { N_F4, 250 }, { N_GS4, 500 }, { N_F4, 350 }, { N_A4, 125 }, { N_C5, 500 }, { N_A4, 375 },
  { N_C5, 125 }, { N_E5, 650 }, { 0, 500 },
  // Second section again
  { N_A5, 500 }, { N_A4, 300 }, { N_A4, 150 }, { N_A5, 500 }, { N_GS5, 325 }, { N_G5, 175 },
  { N_FS5, 125 }, { N_F5, 125 }, { N_FS5, 250 }, { 0, 325 },
  { N_AS4, 250 }, { N_DS5, 500 }, { N_D5, 325 }, { N_CS5, 175 }, { N_C5, 125 }, { N_B4, 125 },
  { N_C5, 250 }, { 0, 350 },
  // Variant 2, ending
  { N_F4, 250 }, { N_GS4, 500 }, { N_F4, 375 }, { N_C5, 125 }, { N_A4, 500 }, { N_F4, 375 },
  { N_C5, 125 }, { N_A4, 650 }
};
const uint8_t MARCH_LEN = sizeof(MARCH) / sizeof(MARCH[0]);
static uint8_t marchIdx = 0;
static unsigned long marchNoteStart = 0;
static bool marchGap = false;

static uint16_t marchHz(uint8_t k)      { return pgm_read_word(&MARCH[k].hz) >> MARCH_OCTAVE_DOWN; }
static unsigned long marchMs(uint8_t k) { return (unsigned long)pgm_read_word(&MARCH[k].ms) * MARCH_TEMPO_PCT / 100; }

static void startNote(uint8_t k) {
  marchIdx       = k;
  marchNoteStart = millis();
  marchGap       = false;
  toneSetHz(marchHz(k));
}

void marchSongStart() { startNote(0); }

// Advances notes. Returns false once the last note has ended (tone off).
bool marchSongUpdate() {
  unsigned long e = millis() - marchNoteStart;
  unsigned long len = marchMs(marchIdx);
  if (e >= len) {
    if (marchIdx + 1 >= MARCH_LEN) {
      toneSetHz(0);
      return false;
    }
    startNote(marchIdx + 1);
  } else if (!marchGap && e + MARCH_NOTE_GAP_MS >= len) {
    toneSetHz(0);
    marchGap = true;
  }
  return true;
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
