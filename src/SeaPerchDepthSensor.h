// SPDX-License-Identifier: MIT

#pragma once

#include <Arduino.h>

class SeaPerchDepthSensor {
 public:
  using MotorOffCallback = void (*)();

  enum class Water : uint8_t {
    Freshwater,
    Saltwater,
  };

  SeaPerchDepthSensor();

  // Call after motor pins have been configured safely. The callback lets the
  // library turn the motor off before every I2C operation.
  void begin(MotorOffCallback motorOff = nullptr,
             Water water = Water::Freshwater);

  // Returns true only when a new, fully validated reading is available.
  // It also performs automatic bounded recovery while returning false.
  bool update();

  bool ready() const;
  float depthCm() const;
  float pressureHpa() const;
  float temperatureC() const;
  float temperatureF() const;
  uint32_t faultCount() const;

 private:
  struct Sample {
    float pressureHpa;
    float temperatureC;
  };

  enum class SampleResult : uint8_t {
    Ok,
    NotReady,
    BusError,
    BadValue,
  };

  void stopMotor() const;
  void prepareForSensorAccess() const;
  void beginWire();
  static bool timeReached(uint32_t now, uint32_t deadline);
  static void driveLineLow(uint8_t pin);
  static void releaseLine(uint8_t pin);
  static bool waitForLineHigh(uint8_t pin, uint32_t timeoutUs);
  bool clearI2cBus();

  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t firstRegister, uint8_t *destination,
                     size_t length);
  bool readRegister(uint8_t reg, uint8_t &value);
  bool writeRegisterChecked(uint8_t reg, uint8_t value, uint8_t mask);
  bool registerMatches(uint8_t reg, uint8_t expected, uint8_t mask);
  bool sensorConfigurationIsValid();
  bool configureSensorBounded();

  SampleResult readSensorSample(Sample &sample);
  SampleResult readFreshSample(Sample &sample, uint32_t timeoutMs);
  bool recoverSensor(Sample &recoveredSample, bool allowFastResume);
  bool calibrateSurfacePressure(Sample &latestSample);
  bool commitSample(const Sample &sample);
  void markFault(SampleResult reason);

  MotorOffCallback motorOffCallback_;
  float hpaPerCm_;

  bool begun_;
  bool healthy_;
  bool baselineValid_;
  bool awaitingPostResetSample_;
  bool pendingRecoveredSampleValid_;
  bool lastRecoveryUsedFastResume_;

  float surfacePressureHpa_;
  float pressureHpa_;
  float temperatureC_;
  float depthCm_;

  uint32_t nextRecoveryAttemptMs_;
  uint32_t faultCount_;
  SampleResult lastFault_;
  Sample pendingRecoveredSample_;
};
