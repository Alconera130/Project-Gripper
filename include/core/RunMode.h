#pragma once

#include <stdint.h>

namespace gripper {

enum class RunMode : uint8_t {
    SAFE,
    EMG_MONITOR,
    EI_FORWARDER,
    CSV_CAPTURE,
    SERVO_TEST,
    MANUAL_GRIP,
    EMG_GRIP,
    MODEL_MONITOR,
    MODEL_CONTROL,
};

enum class CalibrationKind : uint8_t { NONE, IDLE, MAXIMUM };

const char *modeName(RunMode mode);

}  // namespace gripper
