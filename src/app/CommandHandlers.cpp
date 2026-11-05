#include "app/GripperApp.h"

#include "EdgeImpulseBridge.h"
#include "GripperConfig.h"
#include "core/TextUtils.h"
#include "ml/EdgeImpulseModel.h"

namespace gripper {

void GripperApp::handleEmgCommand(char *argument, char *kind,
                                                                    char *durationText) {
    if (equalsIgnoreCase(argument, "start")) {
        const bool canceled = emg_.cancelCalibration();
        servos_.disarm();
        mode_ = RunMode::EMG_MONITOR;
        lastMonitorPrintMs_ = 0;
        Serial.println(canceled
                                              ? "Calibration canceled. EMG monitor started. Send 'emg stop' to leave it."
                                              : "EMG monitor started. Send 'emg stop' to leave it.");
    } else if (equalsIgnoreCase(argument, "stop")) {
        const bool canceled = emg_.cancelCalibration();
        if (mode_ == RunMode::EMG_MONITOR) {
            mode_ = RunMode::SAFE;
        }
        Serial.println(canceled ? "EMG calibration canceled." : "EMG monitor stopped.");
    } else if (equalsIgnoreCase(argument, "cal")) {
        long seconds = 0;
        if (!parseLong(durationText, &seconds) || kind == nullptr ||
                (!equalsIgnoreCase(kind, "idle") && !equalsIgnoreCase(kind, "max"))) {
            Serial.println("Usage: emg cal idle <seconds> OR emg cal max <seconds>");
            return;
        }
        const CalibrationKind calibrationKind = equalsIgnoreCase(kind, "idle")
                                                                                                ? CalibrationKind::IDLE
                                                                                                : CalibrationKind::MAXIMUM;
        if (!emg_.beginCalibration(calibrationKind,
                                                              static_cast<uint32_t>(seconds))) {
            Serial.println("ERROR: calibration duration must be 1..30 seconds.");
            return;
        }
        servos_.disarm();
        mode_ = RunMode::SAFE;
        Serial.printf("CALIBRATING %s for %ld s. %s\n",
                                    calibrationKind == CalibrationKind::IDLE ? "idle" : "maximum",
                                    seconds,
                                    calibrationKind == CalibrationKind::IDLE
                                            ? "Relax the measured muscles and keep electrodes still."
                                            : "Contract the target muscle repeatedly; do not move servos.");
    } else if (equalsIgnoreCase(argument, "progress")) {
        printCalibrationProgress();
    } else {
        Serial.println("Usage: emg start|stop|progress|cal idle <s>|cal max <s>");
    }
}

void GripperApp::handleServoCommand(char *action, char *first, char *second) {
    const int finger = parseFinger(first);
    if (equalsIgnoreCase(action, "set-open") ||
            equalsIgnoreCase(action, "set-close")) {
        long pulse = 0;
        if (finger < 0 || !parseLong(second, &pulse) ||
                pulse < kServoPulseFloorUs || pulse > kServoPulseCeilingUs) {
            Serial.println("Usage: servo set-open|set-close <finger> <1000..2000>");
            return;
        }
        PersistentSettings &stored = settings_.data();
        if (equalsIgnoreCase(action, "set-open")) {
            stored.servoOpenUs[finger] = static_cast<uint16_t>(pulse);
        } else {
            stored.servoCloseUs[finger] = static_cast<uint16_t>(pulse);
        }
        Serial.printf("SET %s %s=%ld us (use 'settings save' to persist).\n",
                                    kFingerNames[finger], action, pulse);
        return;
    }
    if (finger < 0) {
        Serial.println("ERROR: choose finger 1..5 or thumb/index/middle/ring/pinky.");
        return;
    }
    if (!requireArmed()) {
        return;
    }
    if (equalsIgnoreCase(action, "pulse")) {
        long pulse = 0;
        if (!parseLong(second, &pulse) || pulse < kServoPulseFloorUs ||
                pulse > kServoPulseCeilingUs) {
            Serial.println("Usage: servo pulse <finger> <1000..2000>");
            return;
        }
        mode_ = RunMode::SERVO_TEST;
        servos_.setPulse(finger, static_cast<uint16_t>(pulse));
        Serial.printf("TEST %s -> %ld us (tendon disconnected only).\n",
                                    kFingerNames[finger], pulse);
    } else if (equalsIgnoreCase(action, "open") ||
                          equalsIgnoreCase(action, "close")) {
        if (!requireServoCalibration()) {
            return;
        }
        const PersistentSettings &stored = settings_.data();
        mode_ = RunMode::SERVO_TEST;
        servos_.setPulse(finger, equalsIgnoreCase(action, "open")
                                                                  ? stored.servoOpenUs[finger]
                                                                  : stored.servoCloseUs[finger]);
        Serial.printf("TEST %s -> %s target.\n", kFingerNames[finger], action);
    } else {
        Serial.println("Usage: servo pulse|open|close|set-open|set-close <finger> [us]");
    }
}

void GripperApp::handleGripCommand(char *action, char *first, char *second) {
    if (!requireArmed() || !requireServoCalibration()) {
        return;
    }
    if (equalsIgnoreCase(action, "open") || equalsIgnoreCase(action, "close")) {
        mode_ = RunMode::MANUAL_GRIP;
        servos_.setAllFingersPercent(equalsIgnoreCase(action, "open") ? 0 : 100);
        Serial.printf("GRIP %s requested.\n", action);
        return;
    }
    if (equalsIgnoreCase(action, "all")) {
        long percent = 0;
        if (!parseLong(first, &percent) || percent < 0 || percent > 100) {
            Serial.println("Usage: grip all <0..100>");
            return;
        }
        mode_ = RunMode::MANUAL_GRIP;
        servos_.setAllFingersPercent(static_cast<uint8_t>(percent));
        Serial.printf("GRIP all -> %ld%% closed.\n", percent);
        return;
    }
    if (equalsIgnoreCase(action, "finger")) {
        const int finger = parseFinger(first);
        long percent = 0;
        if (finger < 0 || !parseLong(second, &percent) || percent < 0 ||
                percent > 100) {
            Serial.println("Usage: grip finger <finger> <0..100>");
            return;
        }
        mode_ = RunMode::MANUAL_GRIP;
        servos_.setFingerPercent(finger, static_cast<uint8_t>(percent));
        Serial.printf("GRIP %s -> %ld%% closed.\n", kFingerNames[finger],
                                    percent);
        return;
    }
    Serial.println("Usage: grip open|close|all <0..100>|finger <finger> <0..100>");
}

void GripperApp::handleAutoCommand(char *argument) {
    if (equalsIgnoreCase(argument, "on")) {
        if (!requireArmed() || !requireServoCalibration() ||
                !requireEmgCalibration()) {
            return;
        }
        autoGripPercent_ = 0;
        mode_ = RunMode::EMG_GRIP;
        servos_.keepAlive();
        Serial.println("EMG control armed: flex dominates=close, extend dominates=open.");
    } else if (equalsIgnoreCase(argument, "off")) {
        if (mode_ == RunMode::EMG_GRIP) {
            mode_ = RunMode::SAFE;
        }
        Serial.println("EMG control stopped. Servos remain armed until 'disarm'.");
    } else {
        Serial.println("Usage: auto emg on|off");
    }
}

void GripperApp::handleModelCommand(char *argument) {
    if (equalsIgnoreCase(argument, "status")) {
        Serial.printf("MODEL library=%s inference=%s actuation=%s\n",
                                    model::libraryFound() ? "FOUND" : "NOT_FOUND",
                                    model::inferenceEnabled() ? "ENABLED" : "DISABLED",
                                    model::actuationEnabled() ? "ENABLED" : "DISABLED");
        return;
    }
#if GRIPPER_ENABLE_EDGE_IMPULSE
    if (equalsIgnoreCase(argument, "start")) {
        const bool canceled = emg_.cancelCalibration();
        servos_.disarm();
        model::resetWindow();
        mode_ = RunMode::MODEL_MONITOR;
        Serial.println(canceled
                                              ? "EMG calibration canceled. MODEL monitor started; it will print label/confidence only."
                                              : "MODEL monitor started; it will print label/confidence only.");
    } else if (equalsIgnoreCase(argument, "control")) {
        if (!model::actuationEnabled()) {
            Serial.println("ERROR: set kEnableModelActuation=true only after bench validation.");
        } else if (!requireArmed() || !requireServoCalibration()) {
            return;
        } else {
            model::resetWindow();
            mode_ = RunMode::MODEL_CONTROL;
            Serial.println("MODEL control started (guarded confidence/agreement/dwell filters active).");
        }
    } else if (equalsIgnoreCase(argument, "stop")) {
        if (mode_ == RunMode::MODEL_MONITOR || mode_ == RunMode::MODEL_CONTROL) {
            mode_ = RunMode::SAFE;
        }
        Serial.println("MODEL mode stopped.");
    } else {
        Serial.println("Usage: model status|start|control|stop");
    }
#else
    (void)argument;
    Serial.println("MODEL not compiled. Add the exported library and set GRIPPER_ENABLE_EDGE_IMPULSE to 1.");
#endif
}

}  // namespace gripper
