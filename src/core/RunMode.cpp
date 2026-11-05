#include "core/RunMode.h"

namespace gripper {

const char *modeName(RunMode mode) {
    switch (mode) {
        case RunMode::SAFE:
            return "SAFE";
        case RunMode::EMG_MONITOR:
            return "EMG_MONITOR";
        case RunMode::EI_FORWARDER:
            return "EI_FORWARDER";
        case RunMode::CSV_CAPTURE:
            return "CSV_CAPTURE";
        case RunMode::SERVO_TEST:
            return "SERVO_TEST";
        case RunMode::MANUAL_GRIP:
            return "MANUAL_GRIP";
        case RunMode::EMG_GRIP:
            return "EMG_GRIP";
        case RunMode::MODEL_MONITOR:
            return "MODEL_MONITOR";
        case RunMode::MODEL_CONTROL:
            return "MODEL_CONTROL";
    }
    return "UNKNOWN";
}

}  // namespace gripper
