#pragma once

#include <Arduino.h>

// Change only the values in this file when you move wires or calibrate the
// mechanism. Servo travel is deliberately uncalibrated by default: open and
// close have the same pulse width, so a newly flashed board cannot move a
// cable until it has been calibrated on a bench.
namespace gripper {

constexpr uint32_t kSerialBaud = 115200;

constexpr uint8_t kFingerCount = 5;
constexpr uint8_t kServoPins[kFingerCount] = {15, 2, 4, 16, 17};
constexpr const char *kFingerNames[kFingerCount] = {
        "thumb", "index", "middle", "ring", "pinky"};

// Use ADC1 pins. ADC2 is shared with Wi-Fi on the original ESP32 and is best
// avoided for analogue sensors. GPIO 34/35 are input-only, which is ideal here.
constexpr uint8_t kEmgChannelCount = 2;
constexpr uint8_t kEmgPins[kEmgChannelCount] = {34, 35};
constexpr const char *kEmgNames[kEmgChannelCount] = {"flex", "extend"};

constexpr uint16_t kEmgSampleRateHz = 200;
constexpr uint32_t kEmgSamplePeriodUs = 1000000UL / kEmgSampleRateHz;
constexpr float kEmgFilterAlpha = 0.20F;

constexpr uint16_t kServoFrequencyHz = 50;
constexpr uint8_t kServoResolutionBits = 16;
constexpr uint16_t kServoPulseFloorUs = 1000;
constexpr uint16_t kServoPulseCeilingUs = 2000;
constexpr uint16_t kServoNeutralUs = 1500;
constexpr uint16_t kMinimumServoTravelUs = 40;
constexpr uint16_t kServoSlewUsPerSecond = 120;
constexpr uint16_t kServoUpdateIntervalMs = 20;
constexpr uint32_t kServoSessionTimeoutMs = 120000;
constexpr uint16_t kEmgMonitorPeriodMs = 100;

// Initial values stored if no calibration has been saved. Do not change them
// to a moving position; calibrate with the serial commands instead.
constexpr uint16_t kDefaultOpenUs = kServoNeutralUs;
constexpr uint16_t kDefaultCloseUs = kServoNeutralUs;

constexpr uint16_t kAutoActivationPermille = 160;
constexpr uint16_t kAutoDominancePermille = 80;
constexpr uint16_t kAutoReleasePermille = 80;

// Keep 0 until an exported Edge Impulse C++ library has been added under
// lib/edge_impulse and the classifier has been checked with the mechanism
// disconnected. See README.md before enabling it.
#ifndef GRIPPER_ENABLE_EDGE_IMPULSE
#define GRIPPER_ENABLE_EDGE_IMPULSE 0
#endif
constexpr bool kEnableEdgeImpulse = (GRIPPER_ENABLE_EDGE_IMPULSE != 0);
constexpr bool kEnableModelActuation = false;

constexpr uint32_t kSettingsMagic = 0x47525052UL;  // "GRPR"
constexpr uint16_t kSettingsVersion = 1;

}  // namespace gripper
