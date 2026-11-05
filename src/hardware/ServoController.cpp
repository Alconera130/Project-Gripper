#include "hardware/ServoController.h"

// ESP32 Arduino Core 3 uses pin-based LEDC calls, while Core 2 uses channels.
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

namespace gripper {

ServoController::ServoController(SettingsStore &settings) : settings_(settings) {}

bool ServoController::arm() {
    if (armed_) {
        return true;
    }
    for (uint8_t finger = 0; finger < kFingerCount; ++finger) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        if (!ledcAttach(kServoPins[finger], kServoFrequencyHz,
                                        kServoResolutionBits)) {
            Serial.printf("ERROR: could not attach PWM to GPIO %u.\n",
                                        kServoPins[finger]);
            disarm();
            return false;
        }
#else
        ledcSetup(finger, kServoFrequencyHz, kServoResolutionBits);
        ledcAttachPin(kServoPins[finger], finger);
#endif
        servos_[finger].attached = true;
        servos_[finger].currentUs = kServoNeutralUs;
        servos_[finger].targetUs = kServoNeutralUs;
        writePulse(finger, kServoNeutralUs);
    }
    armed_ = true;
    lastUpdateMs_ = millis();
    lastCommandMs_ = millis();
    return true;
}

void ServoController::disarm() {
    for (uint8_t finger = 0; finger < kFingerCount; ++finger) {
        if (servos_[finger].attached) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
            ledcWrite(kServoPins[finger], 0);
            ledcDetach(kServoPins[finger]);
#else
            ledcWrite(finger, 0);
            ledcDetachPin(kServoPins[finger]);
#endif
        }
        servos_[finger].attached = false;
        servos_[finger].currentUs = kServoNeutralUs;
        servos_[finger].targetUs = kServoNeutralUs;
    }
    armed_ = false;
}

bool ServoController::armed() const { return armed_; }

void ServoController::update() {
    if (!armed_) {
        return;
    }
    const uint32_t now = millis();
    const uint32_t elapsed = now - lastUpdateMs_;
    if (elapsed < kServoUpdateIntervalMs) {
        return;
    }
    lastUpdateMs_ = now;
    uint16_t maxStep = static_cast<uint16_t>(
            (static_cast<uint32_t>(kServoSlewUsPerSecond) * elapsed) / 1000UL);
    if (maxStep == 0) {
        maxStep = 1;
    }

    for (uint8_t finger = 0; finger < kFingerCount; ++finger) {
        uint16_t current = servos_[finger].currentUs;
        const uint16_t target = servos_[finger].targetUs;
        if (current < target) {
            current = current + maxStep >= target
                                        ? target
                                        : static_cast<uint16_t>(current + maxStep);
        } else if (current > target) {
            const uint16_t difference = current - target;
            const uint16_t step = maxStep >= difference ? difference : maxStep;
            current = static_cast<uint16_t>(current - step);
        }
        if (current != servos_[finger].currentUs) {
            servos_[finger].currentUs = current;
            writePulse(finger, current);
        }
    }
}

void ServoController::setPulse(uint8_t finger, uint16_t pulseUs) {
    if (finger >= kFingerCount || !armed_) {
        return;
    }
    servos_[finger].targetUs = clampPulse(pulseUs);
    keepAlive();
}

void ServoController::setFingerPercent(uint8_t finger, uint8_t percent) {
    if (finger >= kFingerCount) {
        return;
    }
    if (percent > 100) {
        percent = 100;
    }
    const PersistentSettings &limits = settings_.data();
    const long target = limits.servoOpenUs[finger] +
                                            ((static_cast<long>(limits.servoCloseUs[finger]) -
                                                limits.servoOpenUs[finger]) *
                                              percent) /
                                                    100L;
    setPulse(finger, clampPulse(target));
}

void ServoController::setAllFingersPercent(uint8_t percent) {
    for (uint8_t finger = 0; finger < kFingerCount; ++finger) {
        setFingerPercent(finger, percent);
    }
}

void ServoController::keepAlive() { lastCommandMs_ = millis(); }

bool ServoController::sessionTimedOut() const {
    return armed_ && millis() - lastCommandMs_ >= kServoSessionTimeoutMs;
}

const ServoRuntime &ServoController::runtime(uint8_t finger) const {
    static const ServoRuntime kEmpty{};
    return finger < kFingerCount ? servos_[finger] : kEmpty;
}

uint16_t ServoController::clampPulse(long pulseUs) {
    if (pulseUs < kServoPulseFloorUs) {
        return kServoPulseFloorUs;
    }
    if (pulseUs > kServoPulseCeilingUs) {
        return kServoPulseCeilingUs;
    }
    return static_cast<uint16_t>(pulseUs);
}

void ServoController::writePulse(uint8_t finger, uint16_t pulseUs) {
    if (finger >= kFingerCount || !servos_[finger].attached) {
        return;
    }
    const uint32_t periodUs = 1000000UL / kServoFrequencyHz;
    const uint32_t maxDuty = (1UL << kServoResolutionBits) - 1UL;
    const uint32_t duty = static_cast<uint32_t>(
            (static_cast<uint64_t>(pulseUs) * maxDuty) / periodUs);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(kServoPins[finger], duty);
#else
    ledcWrite(finger, duty);
#endif
}

}  // namespace gripper
