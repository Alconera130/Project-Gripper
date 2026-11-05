#include "app/GripperApp.h"

#include "GripperConfig.h"
#include "ml/EdgeImpulseModel.h"

namespace gripper {

void GripperApp::setup() {
    Serial.begin(kSerialBaud);
    delay(250);

    emg_.begin();
    const bool loaded = settings_.load();
    Serial.println("\nHand Rehab Gripper firmware v1");
    Serial.println("SAFE BOOT: servo PWM is detached until 'arm BENCH_ONLY'.");
    Serial.println(loaded ? "Saved calibration loaded."
                                                : "No valid saved calibration; neutral defaults loaded.");
    Serial.println("Type 'help' for commands. Do not attach EMG SIG directly unless it is proven <= 3.3 V.");
}

void GripperApp::loop() {
    readSerialCommands();
    const uint32_t nowUs = micros();
    if (nowUs - lastSampleUs_ >= kEmgSamplePeriodUs) {
        lastSampleUs_ = nowUs;
        sampleEmg();
    }
    updateEmgMonitor();
    servos_.update();
    updateFailsafe();
}

void GripperApp::sampleEmg() {
    printCalibrationResult(emg_.sample());
    const EmgReading &flex = emg_.reading(0);
    const EmgReading &extend = emg_.reading(1);
    if (mode_ == RunMode::EI_FORWARDER) {
        // The data forwarder must receive only numeric, comma-separated rows.
        Serial.printf("%u,%u\n", flex.raw, extend.raw);
    } else if (mode_ == RunMode::CSV_CAPTURE) {
        const uint32_t timestampMs = (micros() - captureStartedUs_) / 1000UL;
        Serial.printf("%lu,%u,%u\n", static_cast<unsigned long>(timestampMs),
                                    flex.raw, extend.raw);
    }

    updateEmgControl();
    if (mode_ == RunMode::MODEL_MONITOR) {
        model::pushSample(flex.raw, extend.raw, model::RunMode::MONITOR, servos_,
                                            settings_);
    } else if (mode_ == RunMode::MODEL_CONTROL) {
        model::pushSample(flex.raw, extend.raw, model::RunMode::CONTROL, servos_,
                                            settings_);
    }
}

void GripperApp::updateEmgMonitor() {
    if (mode_ != RunMode::EMG_MONITOR ||
            millis() - lastMonitorPrintMs_ < kEmgMonitorPeriodMs) {
        return;
    }
    lastMonitorPrintMs_ = millis();
    const EmgReading &flex = emg_.reading(0);
    const EmgReading &extend = emg_.reading(1);
    Serial.printf(
            "EMG flex raw=%u filtered=%.1f norm=%u | extend raw=%u filtered=%.1f norm=%u\n",
            flex.raw, flex.filtered, emg_.normalized(0), extend.raw, extend.filtered,
            emg_.normalized(1));
}

void GripperApp::updateEmgControl() {
    if (mode_ != RunMode::EMG_GRIP || !servos_.armed()) {
        return;
    }
    const uint16_t flex = emg_.normalized(0);
    const uint16_t extend = emg_.normalized(1);
    uint8_t target = autoGripPercent_;
    if (flex >= kAutoActivationPermille &&
            flex >= extend + kAutoDominancePermille) {
        target = static_cast<uint8_t>(constrain(
                map(flex, kAutoActivationPermille, 1000, 10, 100), 0, 100));
    } else if (extend >= kAutoActivationPermille &&
                          extend >= flex + kAutoDominancePermille) {
        target = static_cast<uint8_t>(100 - constrain(
                map(extend, kAutoActivationPermille, 1000, 10, 100), 0, 100));
    } else {
        return;
    }
    if (target != autoGripPercent_) {
        autoGripPercent_ = target;
        servos_.setAllFingersPercent(target);
    }
}

void GripperApp::updateFailsafe() {
    if (!servos_.sessionTimedOut()) {
        return;
    }
    servos_.disarm();
    mode_ = RunMode::SAFE;
    Serial.println("FAILSAFE: servo session timed out; PWM outputs detached.");
}

void GripperApp::startEiForwarder() {
    // Do not let a calibration completion message corrupt a forwarder stream.
    emg_.cancelCalibration();
    servos_.disarm();
    mode_ = RunMode::EI_FORWARDER;
    captureStartedUs_ = micros();
}

void GripperApp::startCsvCapture() {
    emg_.cancelCalibration();
    servos_.disarm();
    mode_ = RunMode::CSV_CAPTURE;
    captureStartedUs_ = micros();
    Serial.println("timestamp,emg_env_flex,emg_env_extend");
}

void GripperApp::stopCapture() {
    if (mode_ == RunMode::EI_FORWARDER || mode_ == RunMode::CSV_CAPTURE) {
        mode_ = RunMode::SAFE;
    }
}

void GripperApp::setSafeMode(const char *reason) {
    emg_.cancelCalibration();
    servos_.disarm();
    mode_ = RunMode::SAFE;
    if (reason != nullptr) {
        Serial.println(reason);
    }
}

bool GripperApp::requireArmed() {
    if (servos_.armed()) {
        return true;
    }
    Serial.println("ERROR: servos are disarmed. First use: arm BENCH_ONLY");
    return false;
}

bool GripperApp::requireServoCalibration() {
    if (settings_.servosCalibrated()) {
        return true;
    }
    Serial.println("ERROR: servo open/close limits are not calibrated. Use 'servo set-open' and 'servo set-close' first.");
    return false;
}

bool GripperApp::requireEmgCalibration() {
    if (settings_.emgCalibrationValid(0) && settings_.emgCalibrationValid(1)) {
        return true;
    }
    Serial.println("ERROR: EMG baseline/maximum are missing. Run 'emg cal idle 5' then 'emg cal max 5'.");
    return false;
}

}  // namespace gripper
