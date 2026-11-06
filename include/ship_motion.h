#pragma once
#include <Arduino.h>
#include "ship_config.h"

/*****  Space Ships — shared motion code (live + bench_test)  *****
 *  Driver control, limit switches, trip speed profile.
 *  One copy, so a ship tuned on the bench moves the same way live.
 *
 *  DRV8871 (TI datasheet §7.3.1, Table 1): 1/0 forward, 0/1 reverse, 1/1 brake, 0/0 coast.
 *  PWM = drive/brake: TI says PWM "typically works best to switch between driving and braking".
 *  So the input toward the target is held HIGH and the other input is PWM'd (HIGH = brake).
 *******************************************************************/

enum Dir { TO_POS1, TO_POS2 };

struct LimitSwitch {
  uint8_t pin;
  uint8_t activeLevel;      // Pin level at the limit
  bool active;              // Debounced: true = ship is at this limit
  bool lastRaw;
  unsigned long lastChange; // When the raw reading last changed
};

// Calibration values for one ship (see ship_config.h)
struct ShipTuning {
  uint8_t mode;
  uint8_t steadyDuty;
  unsigned long steadyTripMs;
  uint8_t minDuty;
  unsigned long minTripMs;
  // MODE_SURGE (speeds in %)
  uint8_t startPct;             // Duty at motor-on
  unsigned long takeOffRampMs;  // Start -> take-off
  uint8_t takeOffPct;
  unsigned long hyperRampMs;    // Take-off -> hyper
  uint8_t hyperPct;
  unsigned long hyperHoldMs;    // Time at hyper
  unsigned long cruiseRampMs;   // Hyper -> cruise
  uint8_t cruisePct;
  unsigned long limitClearMs;  // 0 = not measured
};

struct ShipIO {
  uint8_t id;
  uint8_t inToPos1;
  uint8_t inToPos2;
  LimitSwitch pos1;
  LimitSwitch pos2;
  int16_t lastCmd;          // Last drive command, so PWM is only rewritten on change
};

extern ShipIO shipIO[2];
extern const ShipTuning SHIP_TUNING[2];  // From ship_config.h

// Limits
void limitUpdate(LimitSwitch &sw);
void shipUpdateLimits(ShipIO &s);
bool bothLimits(const ShipIO &s);
LimitSwitch &aheadOf(ShipIO &s, Dir d);
LimitSwitch &behindOf(ShipIO &s, Dir d);

// Motor
void shipBegin(ShipIO &s);                        // Outputs to brake, limits read
void motorBrake(ShipIO &s);
void motorDrive(ShipIO &s, Dir d, uint8_t duty);  // duty 0 = brake, 255 = full

// Speed profile, elapsed = ms since the trip started
uint8_t tripDuty(const ShipTuning &t, unsigned long elapsed);   // Per t.mode
unsigned long clearMs(const ShipTuning &t);  // Wrong-way check time for this ship's mode

// Tone mode (bench Imperial March): the TCA0 PWM frequency is the note. D5, D9, D10 share it.
void toneSetHz(uint16_t hz);                          // 0 = silent (normal PWM frequency)
void motorDriveTone(ShipIO &s, Dir d, uint8_t duty);  // As motorDrive, PWM on a TCA0 pin so it sings
void toneEnd();                                       // Back to normal PWM. Brake the motors first.
void marchSongStart();                                // Imperial March, first note
bool marchSongUpdate();                               // Call every loop. False once the song has ended.
