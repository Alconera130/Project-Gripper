#pragma once

#include <Arduino.h>

namespace gripper {
class ServoController;
class SettingsStore;

namespace model {

enum class RunMode : uint8_t { MONITOR, CONTROL };

bool libraryFound();
bool inferenceEnabled();
bool actuationEnabled();
void resetWindow();
void pushSample(uint16_t flex, uint16_t extend, RunMode mode,
                                ServoController &servos, const SettingsStore &settings);

}  // namespace model
}  // namespace gripper
