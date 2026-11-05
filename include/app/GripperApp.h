#pragma once

#include "core/RunMode.h"
#include "hardware/EmgController.h"
#include "hardware/ServoController.h"
#include "settings/SettingsStore.h"

namespace gripper {

class GripperApp {
    public:
    void setup();
    void loop();

    private:
    void sampleEmg();
    void updateEmgMonitor();
    void updateEmgControl();
    void updateFailsafe();

    void startEiForwarder();
    void startCsvCapture();
    void stopCapture();
    void setSafeMode(const char *reason);

    bool requireArmed();
    bool requireServoCalibration();
    bool requireEmgCalibration();

    void printStatus();
    void printHelp();
    void printSettings();
    void printCalibrationProgress();
    void printCalibrationResult(CalibrationResult result);

    void handleCommand(char *line);
    void handleEmgCommand(char *argument, char *kind, char *durationText);
    void handleServoCommand(char *action, char *first, char *second);
    void handleGripCommand(char *action, char *first, char *second);
    void handleAutoCommand(char *argument);
    void handleModelCommand(char *argument);
    void readSerialCommands();

    SettingsStore settings_;
    ServoController servos_{settings_};
    EmgController emg_{settings_};
    RunMode mode_ = RunMode::SAFE;
    uint32_t lastSampleUs_ = 0;
    uint32_t lastMonitorPrintMs_ = 0;
    uint32_t captureStartedUs_ = 0;
    uint8_t autoGripPercent_ = 0;
    char commandBuffer_[128]{};
    size_t commandLength_ = 0;
};

}  // namespace gripper
