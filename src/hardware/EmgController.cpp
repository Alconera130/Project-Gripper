#include "hardware/EmgController.h"

#include <math.h>
#include <string.h>

namespace gripper {

EmgController::EmgController(SettingsStore &settings) : settings_(settings) {}

void EmgController::begin() {
    analogReadResolution(12);
    for (uint8_t channel = 0; channel < kEmgChannelCount; ++channel) {
        pinMode(kEmgPins[channel], INPUT);
        analogSetPinAttenuation(kEmgPins[channel], ADC_11db);
    }
}

CalibrationResult EmgController::sample() {
    for (uint8_t channel = 0; channel < kEmgChannelCount; ++channel) {
        const int adc = analogRead(kEmgPins[channel]);
        readings_[channel].raw = static_cast<uint16_t>(constrain(adc, 0, 4095));
        if (!initialized_) {
            readings_[channel].filtered = readings_[channel].raw;
        } else {
            readings_[channel].filtered +=
                    kEmgFilterAlpha * (readings_[channel].raw - readings_[channel].filtered);
        }
    }
    initialized_ = true;
    return updateCalibration();
}

const EmgReading &EmgController::reading(uint8_t channel) const {
    static const EmgReading kEmpty{};
    return channel < kEmgChannelCount ? readings_[channel] : kEmpty;
}

uint16_t EmgController::normalized(uint8_t channel) const {
    if (channel >= kEmgChannelCount || !settings_.emgCalibrationValid(channel)) {
        return 0;
    }
    const PersistentSettings &stored = settings_.data();
    const float numerator = readings_[channel].filtered - stored.emgBaseline[channel];
    const float denominator = stored.emgPeak[channel] - stored.emgBaseline[channel];
    long value = lroundf((numerator * 1000.0F) / denominator);
    if (value < 0) {
        value = 0;
    }
    if (value > 1000) {
        value = 1000;
    }
    return static_cast<uint16_t>(value);
}

bool EmgController::beginCalibration(CalibrationKind kind, uint32_t seconds) {
    if ((kind != CalibrationKind::IDLE && kind != CalibrationKind::MAXIMUM) ||
            seconds == 0 || seconds > 30) {
        return false;
    }
    memset(&calibration_, 0, sizeof(calibration_));
    calibration_.kind = kind;
    calibration_.startedMs = millis();
    calibration_.durationMs = seconds * 1000UL;
    return true;
}

bool EmgController::calibrationActive() const {
    return calibration_.kind != CalibrationKind::NONE;
}

CalibrationKind EmgController::calibrationKind() const {
    return calibration_.kind;
}

uint32_t EmgController::calibrationRemainingMs() const {
    if (!calibrationActive()) {
        return 0;
    }
    const uint32_t elapsed = millis() - calibration_.startedMs;
    return elapsed >= calibration_.durationMs ? 0
                                                                                          : calibration_.durationMs - elapsed;
}

bool EmgController::cancelCalibration() {
    if (!calibrationActive()) {
        return false;
    }
    calibration_.kind = CalibrationKind::NONE;
    return true;
}

CalibrationResult EmgController::updateCalibration() {
    if (!calibrationActive()) {
        return CalibrationResult::NONE;
    }
    for (uint8_t channel = 0; channel < kEmgChannelCount; ++channel) {
        const uint16_t value = readings_[channel].raw;
        calibration_.sum[channel] += value;
        if (value > calibration_.maximum[channel]) {
            calibration_.maximum[channel] = value;
        }
    }
    ++calibration_.count;
    if (millis() - calibration_.startedMs < calibration_.durationMs) {
        return CalibrationResult::NONE;
    }

    const CalibrationKind completedKind = calibration_.kind;
    calibration_.kind = CalibrationKind::NONE;
    if (calibration_.count == 0) {
        return CalibrationResult::FAILED;
    }

    PersistentSettings &stored = settings_.data();
    if (completedKind == CalibrationKind::IDLE) {
        for (uint8_t channel = 0; channel < kEmgChannelCount; ++channel) {
            stored.emgBaseline[channel] = static_cast<uint16_t>(
                    calibration_.sum[channel] / calibration_.count);
        }
        return CalibrationResult::IDLE_COMPLETE;
    }
    for (uint8_t channel = 0; channel < kEmgChannelCount; ++channel) {
        stored.emgPeak[channel] = calibration_.maximum[channel];
    }
    return CalibrationResult::MAXIMUM_COMPLETE;
}

}  // namespace gripper
