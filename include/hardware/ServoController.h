#pragma once

#include <Arduino.h>

#include "settings/SettingsStore.h"

namespace gripper {

struct ServoRuntime {
    bool attached;
    uint16_t currentUs;
    uint16_t targetUs;
};

class ServoController {
    public:
    explicit ServoController(SettingsStore &settings);

    bool arm();
    void disarm();
    bool armed() const;
    void update();

    void setPulse(uint8_t finger, uint16_t pulseUs);
    void setFingerPercent(uint8_t finger, uint8_t percent);
    void setAllFingersPercent(uint8_t percent);
    void keepAlive();
    bool sessionTimedOut() const;

    const ServoRuntime &runtime(uint8_t finger) const;

    private:
    static uint16_t clampPulse(long pulseUs);
    void writePulse(uint8_t finger, uint16_t pulseUs);

    SettingsStore &settings_;
    ServoRuntime servos_[kFingerCount]{};
    bool armed_ = false;
    uint32_t lastUpdateMs_ = 0;
    uint32_t lastCommandMs_ = 0;
};

}  // namespace gripper
