#include "settings/SettingsStore.h"

#include <Preferences.h>

#include <string.h>

namespace gripper {
namespace {

uint32_t fnv1a(const uint8_t *data, size_t length) {
    uint32_t hash = 2166136261UL;
    for (size_t i = 0; i < length; ++i) {
        hash ^= data[i];
        hash *= 16777619UL;
    }
    return hash;
}

}  // namespace

bool SettingsStore::load() {
    Preferences preferences;
    if (!preferences.begin("gripper", true)) {
        loadFactoryDefaults();
        return false;
    }

    PersistentSettings candidate{};
    if (preferences.getBytesLength("settings") == sizeof(candidate)) {
        preferences.getBytes("settings", &candidate, sizeof(candidate));
    }
    preferences.end();

    if (!isValid(candidate)) {
        loadFactoryDefaults();
        return false;
    }
    settings_ = candidate;
    return true;
}

bool SettingsStore::save() {
    settings_.magic = kSettingsMagic;
    settings_.version = kSettingsVersion;
    settings_.checksum = checksum(settings_);

    Preferences preferences;
    if (!preferences.begin("gripper", false)) {
        return false;
    }
    const size_t written = preferences.putBytes("settings", &settings_,
                                                                                            sizeof(settings_));
    preferences.end();
    return written == sizeof(settings_);
}

bool SettingsStore::factoryReset() {
    Preferences preferences;
    if (!preferences.begin("gripper", false)) {
        return false;
    }
    const bool removed = preferences.remove("settings");
    preferences.end();
    loadFactoryDefaults();
    return removed;
}

PersistentSettings &SettingsStore::data() { return settings_; }

const PersistentSettings &SettingsStore::data() const { return settings_; }

bool SettingsStore::servosCalibrated() const {
    for (uint8_t finger = 0; finger < kFingerCount; ++finger) {
        const int travel = static_cast<int>(settings_.servoCloseUs[finger]) -
                                              static_cast<int>(settings_.servoOpenUs[finger]);
        if (travel > -static_cast<int>(kMinimumServoTravelUs) &&
                travel < static_cast<int>(kMinimumServoTravelUs)) {
            return false;
        }
    }
    return true;
}

bool SettingsStore::emgCalibrationValid(uint8_t channel) const {
    return channel < kEmgChannelCount &&
                  settings_.emgPeak[channel] > settings_.emgBaseline[channel] + 20;
}

void SettingsStore::loadFactoryDefaults() {
    memset(&settings_, 0, sizeof(settings_));
    settings_.magic = kSettingsMagic;
    settings_.version = kSettingsVersion;
    for (uint8_t finger = 0; finger < kFingerCount; ++finger) {
        settings_.servoOpenUs[finger] = kDefaultOpenUs;
        settings_.servoCloseUs[finger] = kDefaultCloseUs;
    }
    settings_.checksum = checksum(settings_);
}

bool SettingsStore::isValid(const PersistentSettings &candidate) const {
    if (candidate.magic != kSettingsMagic ||
            candidate.version != kSettingsVersion ||
            candidate.checksum != checksum(candidate)) {
        return false;
    }
    for (uint8_t finger = 0; finger < kFingerCount; ++finger) {
        if (candidate.servoOpenUs[finger] < kServoPulseFloorUs ||
                candidate.servoOpenUs[finger] > kServoPulseCeilingUs ||
                candidate.servoCloseUs[finger] < kServoPulseFloorUs ||
                candidate.servoCloseUs[finger] > kServoPulseCeilingUs) {
            return false;
        }
    }
    return true;
}

uint32_t SettingsStore::checksum(const PersistentSettings &candidate) const {
    return fnv1a(reinterpret_cast<const uint8_t *>(&candidate),
                              sizeof(PersistentSettings) - sizeof(candidate.checksum));
}

}  // namespace gripper
