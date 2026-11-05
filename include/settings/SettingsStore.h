#pragma once

#include <Arduino.h>

#include "GripperConfig.h"

namespace gripper {

// This layout is stored as one checked record in ESP32 NVS. Do not reorder
// fields without increasing kSettingsVersion in GripperConfig.h.
struct PersistentSettings {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint16_t servoOpenUs[kFingerCount];
    uint16_t servoCloseUs[kFingerCount];
    uint16_t emgBaseline[kEmgChannelCount];
    uint16_t emgPeak[kEmgChannelCount];
    uint32_t checksum;
};

class SettingsStore {
    public:
    bool load();
    bool save();
    bool factoryReset();

    PersistentSettings &data();
    const PersistentSettings &data() const;
    bool servosCalibrated() const;
    bool emgCalibrationValid(uint8_t channel) const;

    private:
    void loadFactoryDefaults();
    bool isValid(const PersistentSettings &candidate) const;
    uint32_t checksum(const PersistentSettings &candidate) const;

    PersistentSettings settings_{};
};

}  // namespace gripper
