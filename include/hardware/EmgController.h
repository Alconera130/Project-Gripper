#pragma once

#include <Arduino.h>

#include "core/RunMode.h"
#include "settings/SettingsStore.h"

namespace gripper {

struct EmgReading {
    uint16_t raw;
    float filtered;
};

enum class CalibrationResult : uint8_t {
    NONE,
    IDLE_COMPLETE,
    MAXIMUM_COMPLETE,
    FAILED,
};

class EmgController {
    public:
    explicit EmgController(SettingsStore &settings);

    void begin();
    CalibrationResult sample();
    const EmgReading &reading(uint8_t channel) const;
    uint16_t normalized(uint8_t channel) const;

    bool beginCalibration(CalibrationKind kind, uint32_t seconds);
    bool calibrationActive() const;
    CalibrationKind calibrationKind() const;
    uint32_t calibrationRemainingMs() const;
    bool cancelCalibration();

    private:
    struct CalibrationJob {
        CalibrationKind kind;
        uint32_t startedMs;
        uint32_t durationMs;
        uint32_t count;
        uint32_t sum[kEmgChannelCount];
        uint16_t maximum[kEmgChannelCount];
    };

    CalibrationResult updateCalibration();

    SettingsStore &settings_;
    EmgReading readings_[kEmgChannelCount]{};
    CalibrationJob calibration_{};
    bool initialized_ = false;
};

}  // namespace gripper
