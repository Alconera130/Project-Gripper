#include "ml/EdgeImpulseModel.h"

#include <string.h>

#include "EdgeImpulseBridge.h"
#include "GripperConfig.h"
#include "core/TextUtils.h"
#include "hardware/ServoController.h"
#include "settings/SettingsStore.h"

namespace gripper {
namespace model {

bool libraryFound() { return GRIPPER_EDGE_IMPULSE_LIBRARY_FOUND != 0; }

bool inferenceEnabled() { return kEnableEdgeImpulse; }

bool actuationEnabled() { return kEnableModelActuation; }

#if GRIPPER_ENABLE_EDGE_IMPULSE
namespace {

float gRing[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]{};
float gSnapshot[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]{};
size_t gWriteIndex = 0;
size_t gValuesCollected = 0;
uint32_t gLastInferenceMs = 0;
char gLastLabel[48]{};
uint8_t gSameLabelCount = 0;
uint32_t gLastActionMs = 0;

constexpr float kMinimumConfidence = 0.85F;
constexpr uint8_t kRequiredAgreement = 3;
constexpr uint32_t kMinimumDwellMs = 500;

int getData(size_t offset, size_t length, float *outPtr) {
    if (offset + length > EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        return -1;
    }
    memcpy(outPtr, gSnapshot + offset, length * sizeof(float));
    return 0;
}

void setActionForLabel(const char *label, ServoController &servos,
                                              const SettingsStore &settings) {
    if (!kEnableModelActuation || !servos.armed() ||
            !settings.servosCalibrated()) {
        return;
    }
    if (equalsIgnoreCase(label, "open") || equalsIgnoreCase(label, "rest")) {
        servos.setAllFingersPercent(0);
        return;
    }
    if (equalsIgnoreCase(label, "close") || equalsIgnoreCase(label, "grip")) {
        servos.setAllFingersPercent(100);
        return;
    }
    for (uint8_t finger = 0; finger < kFingerCount; ++finger) {
        if (startsWithIgnoreCase(label, kFingerNames[finger])) {
            const size_t labelLength = strlen(label);
            const bool openLabel = labelLength >= 4 &&
                                                          equalsIgnoreCase(label + labelLength - 4, "open");
            servos.setFingerPercent(finger, openLabel ? 0 : 100);
            return;
        }
    }
}

void runClassifier(RunMode mode, ServoController &servos,
                                      const SettingsStore &settings) {
    if (gValuesCollected < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        return;
    }
    const uint32_t now = millis();
    if (now - gLastInferenceMs < kMinimumDwellMs) {
        return;
    }
    gLastInferenceMs = now;

    for (size_t i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ++i) {
        gSnapshot[i] = gRing[(gWriteIndex + i) % EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
    }

    signal_t signal{};
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = getData;
    ei_impulse_result_t result{};
    const EI_IMPULSE_ERROR status = run_classifier(&signal, &result, false);
    if (status != EI_IMPULSE_OK) {
        Serial.printf("MODEL ERROR: run_classifier=%d\n", static_cast<int>(status));
        return;
    }

    const char *bestLabel = "";
    float bestValue = -1.0F;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; ++i) {
        if (result.classification[i].value > bestValue) {
            bestValue = result.classification[i].value;
            bestLabel = result.classification[i].label;
        }
    }
    Serial.printf("MODEL label=%s confidence=%.3f\n", bestLabel, bestValue);

    if (bestValue < kMinimumConfidence) {
        gSameLabelCount = 0;
        gLastLabel[0] = '\0';
        return;
    }
    if (equalsIgnoreCase(bestLabel, gLastLabel)) {
        if (gSameLabelCount < 255) {
            ++gSameLabelCount;
        }
    } else {
        strncpy(gLastLabel, bestLabel, sizeof(gLastLabel) - 1);
        gLastLabel[sizeof(gLastLabel) - 1] = '\0';
        gSameLabelCount = 1;
    }
    if (mode == RunMode::CONTROL &&
            gSameLabelCount >= kRequiredAgreement &&
            now - gLastActionMs >= kMinimumDwellMs) {
        setActionForLabel(bestLabel, servos, settings);
        gLastActionMs = now;
    }
}

}  // namespace
#endif

void resetWindow() {
#if GRIPPER_ENABLE_EDGE_IMPULSE
    gWriteIndex = 0;
    gValuesCollected = 0;
    gLastInferenceMs = 0;
    gLastLabel[0] = '\0';
    gSameLabelCount = 0;
    gLastActionMs = 0;
#endif
}

void pushSample(uint16_t flex, uint16_t extend, RunMode mode,
                                ServoController &servos, const SettingsStore &settings) {
#if GRIPPER_ENABLE_EDGE_IMPULSE
    const float values[kEmgChannelCount] = {static_cast<float>(flex),
                                                                                    static_cast<float>(extend)};
    for (uint8_t channel = 0; channel < kEmgChannelCount; ++channel) {
        gRing[gWriteIndex] = values[channel];
        gWriteIndex = (gWriteIndex + 1) % EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        if (gValuesCollected < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            ++gValuesCollected;
        }
    }
    runClassifier(mode, servos, settings);
#else
    (void)flex;
    (void)extend;
    (void)mode;
    (void)servos;
    (void)settings;
#endif
}

}  // namespace model
}  // namespace gripper
