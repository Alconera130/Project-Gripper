#include "app/GripperApp.h"

#include "GripperConfig.h"

namespace gripper {

void GripperApp::printStatus() {
    Serial.printf("MODE=%s | servos=%s | servo_cal=%s | emg_cal=%s\n",
                                modeName(mode_), servos_.armed() ? "ARMED" : "DISARMED",
                                settings_.servosCalibrated() ? "READY" : "MISSING",
                                (settings_.emgCalibrationValid(0) &&
                                  settings_.emgCalibrationValid(1))
                                        ? "READY"
                                        : "MISSING");
    const EmgReading &flex = emg_.reading(0);
    const EmgReading &extend = emg_.reading(1);
    Serial.printf(
            "EMG ADC: flex=%u filtered=%.1f norm=%u | extend=%u filtered=%.1f norm=%u\n",
            flex.raw, flex.filtered, emg_.normalized(0), extend.raw, extend.filtered,
            emg_.normalized(1));
    const PersistentSettings &stored = settings_.data();
    for (uint8_t finger = 0; finger < kFingerCount; ++finger) {
        const ServoRuntime &servo = servos_.runtime(finger);
        Serial.printf("%u:%s pin=%u open=%u close=%u current=%u target=%u\n",
                                    finger + 1, kFingerNames[finger], kServoPins[finger],
                                    stored.servoOpenUs[finger], stored.servoCloseUs[finger],
                                    servo.currentUs, servo.targetUs);
    }
}

void GripperApp::printHelp() {
    Serial.println("\n--- Hand Rehab Gripper commands ---");
    Serial.println("status                         Show EMG, limits, and state");
    Serial.println("arm BENCH_ONLY                 Attach PWM at neutral 1500 us");
    Serial.println("disarm | stop | safe           Detach every servo immediately");
    Serial.println("keepalive                      Reset the 120 s servo-session timer");
    Serial.println("emg start | emg stop           Human-readable two-channel monitor");
    Serial.println("emg cal idle <1..30 s>         Relaxed baseline (servos disarmed)");
    Serial.println("emg cal max <1..30 s>          Repeated contraction peak");
    Serial.println("ei start | ei stop             Headerless 200 Hz flex,extend CSV");
    Serial.println("csv start | csv stop           Timestamped CSV for manual upload");
    Serial.println("servo pulse <finger> <us>      Bench-test 1000..2000 us only");
    Serial.println("servo set-open <finger> <us>   Record calibrated open position");
    Serial.println("servo set-close <finger> <us>  Record calibrated close position");
    Serial.println("servo open|close <finger>      Move one calibrated tendon slowly");
    Serial.println("grip open | grip close         Move all calibrated fingers slowly");
    Serial.println("grip all <0..100>              Set all finger closing percentage");
    Serial.println("grip finger <finger> <0..100> Set one finger closing percentage");
    Serial.println("auto emg on | auto emg off     Two-channel flex/extend control");
    Serial.println("settings show | save | factory RESET");
    Serial.println("model status|start|stop|control (optional Edge Impulse build)");
    Serial.println("Finger may be 1..5 or thumb/index/middle/ring/pinky.");
}

void GripperApp::printSettings() {
    const PersistentSettings &stored = settings_.data();
    Serial.printf("Stored EMG baseline: flex=%u extend=%u; peak: flex=%u extend=%u\n",
                                stored.emgBaseline[0], stored.emgBaseline[1],
                                stored.emgPeak[0], stored.emgPeak[1]);
    for (uint8_t finger = 0; finger < kFingerCount; ++finger) {
        Serial.printf("%s: open=%u close=%u\n", kFingerNames[finger],
                                    stored.servoOpenUs[finger], stored.servoCloseUs[finger]);
    }
}

void GripperApp::printCalibrationProgress() {
    if (!emg_.calibrationActive()) {
        return;
    }
    Serial.printf("CALIBRATING %s: %lu ms remaining\n",
                                emg_.calibrationKind() == CalibrationKind::IDLE ? "idle"
                                                                                                                                  : "max",
                                static_cast<unsigned long>(emg_.calibrationRemainingMs()));
}

void GripperApp::printCalibrationResult(CalibrationResult result) {
    const PersistentSettings &stored = settings_.data();
    switch (result) {
        case CalibrationResult::IDLE_COMPLETE:
            Serial.printf("IDLE CAL COMPLETE: flex=%u extend=%u. Run 'emg cal max 5' next.\n",
                                        stored.emgBaseline[0], stored.emgBaseline[1]);
            break;
        case CalibrationResult::MAXIMUM_COMPLETE:
            Serial.printf("MAX CAL COMPLETE: flex=%u extend=%u. Use 'settings save' to persist.\n",
                                        stored.emgPeak[0], stored.emgPeak[1]);
            break;
        case CalibrationResult::FAILED:
            Serial.println("ERROR: no EMG samples were collected.");
            break;
        case CalibrationResult::NONE:
            break;
    }
}

}  // namespace gripper
