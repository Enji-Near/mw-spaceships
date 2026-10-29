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
  uint8_t accelPct;
  uint8_t decelPct;
  unsigned long decelStartMs;
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
unsigned long accelMs(const ShipTuning &t);
unsigned long decelMs(const ShipTuning &t);
uint8_t accelDuty(const ShipTuning &t, unsigned long elapsed);  // Accel, then hold steady
uint8_t tripDuty(const ShipTuning &t, unsigned long elapsed);   // Per t.mode
