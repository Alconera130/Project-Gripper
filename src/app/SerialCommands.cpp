#include "app/GripperApp.h"

#include <string.h>

#include "core/TextUtils.h"

namespace gripper {

void GripperApp::handleCommand(char *line) {
    line = trim(line);
    if (line == nullptr || *line == '\0') {
        return;
    }
    if (startsWithIgnoreCase(line, "AT+STARTDATAFWD")) {
        startEiForwarder();
        return;
    }
    if (startsWithIgnoreCase(line, "AT+STOPDATAFWD")) {
        stopCapture();
        return;
    }
    char *context = nullptr;
    char *command = strtok_r(line, " \t", &context);
    char *first = strtok_r(nullptr, " \t", &context);
    char *second = strtok_r(nullptr, " \t", &context);
    char *third = strtok_r(nullptr, " \t", &context);

    if (equalsIgnoreCase(command, "help") || equalsIgnoreCase(command, "?")) {
        printHelp();
    } else if (equalsIgnoreCase(command, "status")) {
        printStatus();
    } else if (equalsIgnoreCase(command, "arm")) {
        if (!equalsIgnoreCase(first, "BENCH_ONLY")) {
            Serial.println("Refusing to arm. Disconnect tendons, then send: arm BENCH_ONLY");
        } else if (emg_.calibrationActive()) {
            Serial.println("ERROR: EMG calibration is active. Send 'emg stop' before arming servos.");
        } else if (servos_.arm()) {
            mode_ = RunMode::SAFE;
            Serial.println("ARMED at neutral pulse. Verify physical clearance before any motion command.");
        }
    } else if (equalsIgnoreCase(command, "disarm") ||
                          equalsIgnoreCase(command, "stop") ||
                          equalsIgnoreCase(command, "safe")) {
        setSafeMode("SAFE: all servo PWM outputs detached.");
    } else if (equalsIgnoreCase(command, "keepalive")) {
        if (requireArmed()) {
            servos_.keepAlive();
            Serial.println("Servo session extended.");
        }
    } else if (equalsIgnoreCase(command, "emg")) {
        handleEmgCommand(first, second, third);
    } else if (equalsIgnoreCase(command, "ei")) {
        if (equalsIgnoreCase(first, "start")) {
            startEiForwarder();
        } else if (equalsIgnoreCase(first, "stop")) {
            stopCapture();
        } else {
            Serial.println("Usage: ei start|stop");
        }
    } else if (equalsIgnoreCase(command, "csv")) {
        if (equalsIgnoreCase(first, "start")) {
            startCsvCapture();
        } else if (equalsIgnoreCase(first, "stop")) {
            stopCapture();
        } else {
            Serial.println("Usage: csv start|stop");
        }
    } else if (equalsIgnoreCase(command, "servo")) {
        handleServoCommand(first, second, third);
    } else if (equalsIgnoreCase(command, "grip")) {
        handleGripCommand(first, second, third);
    } else if (equalsIgnoreCase(command, "auto")) {
        if (!equalsIgnoreCase(first, "emg")) {
            Serial.println("Usage: auto emg on|off");
        } else {
            handleAutoCommand(second);
        }
    } else if (equalsIgnoreCase(command, "settings")) {
        if (equalsIgnoreCase(first, "show")) {
            printSettings();
        } else if (equalsIgnoreCase(first, "save")) {
            Serial.println(settings_.save() ? "Settings saved to ESP32."
                                                                            : "ERROR: could not save settings.");
        } else if (equalsIgnoreCase(first, "factory") &&
                              equalsIgnoreCase(second, "RESET")) {
            setSafeMode(nullptr);
            settings_.factoryReset();
            Serial.println("Factory calibration restored (and stored settings removed).");
        } else {
            Serial.println("Usage: settings show|save|factory RESET");
        }
    } else if (equalsIgnoreCase(command, "model")) {
        handleModelCommand(first);
    } else {
        Serial.println("Unknown command. Send 'help'.");
    }
}

void GripperApp::readSerialCommands() {
    while (Serial.available() > 0) {
        const char character = static_cast<char>(Serial.read());
        if (character == '\r') {
            continue;
        }
        if (character == '\n') {
            commandBuffer_[commandLength_] = '\0';
            handleCommand(commandBuffer_);
            commandLength_ = 0;
            continue;
        }
        if ((character == '\b' || character == 127) && commandLength_ > 0) {
            --commandLength_;
            continue;
        }
        if (commandLength_ < sizeof(commandBuffer_) - 1) {
            commandBuffer_[commandLength_++] = character;
        } else {
            commandLength_ = 0;
            Serial.println("ERROR: command too long.");
        }
    }
}

}  // namespace gripper
