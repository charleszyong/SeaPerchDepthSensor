// SPDX-License-Identifier: MIT

#include "SeaPerchDepthSensor.h"

#include <Wire.h>
#include <math.h>

#if !defined(ARDUINO_ARCH_RENESAS_UNO)
#error "SeaPerchDepthSensor requires an Arduino Renesas UNO board core."
#endif

namespace {

// LPS35HW registers and configuration.
constexpr uint8_t kSensorAddress = 0x5D;
constexpr uint8_t kInterruptConfigRegister = 0x0B;
constexpr uint8_t kWhoAmIRegister = 0x0F;
constexpr uint8_t kCtrl1Register = 0x10;
constexpr uint8_t kCtrl2Register = 0x11;
constexpr uint8_t kPressureOffsetLowRegister = 0x18;
constexpr uint8_t kPressureOffsetHighRegister = 0x19;
constexpr uint8_t kStatusRegister = 0x27;
constexpr uint8_t kPressureOutXlRegister = 0x28;
constexpr uint8_t kTemperatureOutLRegister = 0x2B;
constexpr uint8_t kExpectedWhoAmI = 0xB1;

constexpr uint8_t kCtrl1_10HzBdu = 0x22;
constexpr uint8_t kCtrl2AutoIncrement = 0x10;
constexpr uint8_t kCtrl2SoftwareReset = 0x04;
constexpr uint8_t kStatusPressureAndTemperatureReady = 0x03;

constexpr uint32_t kI2cClockHz = 100000UL;
constexpr uint32_t kI2cTimeoutUs = 5000UL;
constexpr uint32_t kSensorResetTimeoutMs = 100UL;
constexpr uint32_t kFreshSampleTimeoutMs = 150UL;
constexpr uint32_t kFreshSamplePollMs = 10UL;
constexpr uint32_t kRecoveryRetryMs = 50UL;
constexpr uint8_t kRecoveryAttemptsPerRun = 2;
constexpr uint8_t kBaselineSampleCount = 16;
constexpr float kMaxBaselineSpreadHpa = 2.0f;

constexpr float kFreshwaterHpaPerCm = 0.9778f;
constexpr float kSaltwaterHpaPerCm = 1.0038f;
constexpr float kMinimumPressureHpa = 260.0f;
constexpr float kMaximumPressureHpa = 1260.0f;
constexpr float kMinimumTemperatureC = -40.0f;
constexpr float kMaximumTemperatureC = 85.0f;
constexpr uint32_t kMotorI2cQuietTimeMs = 5UL;

}  // namespace

SeaPerchDepthSensor::SeaPerchDepthSensor()
    : motorOffCallback_(nullptr),
      hpaPerCm_(kFreshwaterHpaPerCm),
      begun_(false),
      healthy_(false),
      baselineValid_(false),
      awaitingPostResetSample_(false),
      pendingRecoveredSampleValid_(false),
      lastRecoveryUsedFastResume_(false),
      surfacePressureHpa_(0.0f),
      pressureHpa_(0.0f),
      temperatureC_(0.0f),
      depthCm_(0.0f),
      nextRecoveryAttemptMs_(0),
      faultCount_(0),
      lastFault_(SampleResult::BusError),
      pendingRecoveredSample_{0.0f, 0.0f} {}

void SeaPerchDepthSensor::begin(MotorOffCallback motorOff, Water water) {
  motorOffCallback_ = motorOff;
  hpaPerCm_ = water == Water::Saltwater ? kSaltwaterHpaPerCm
                                       : kFreshwaterHpaPerCm;

  begun_ = true;
  healthy_ = false;
  baselineValid_ = false;
  awaitingPostResetSample_ = false;
  pendingRecoveredSampleValid_ = false;
  lastRecoveryUsedFastResume_ = false;
  surfacePressureHpa_ = 0.0f;
  pressureHpa_ = 0.0f;
  temperatureC_ = 0.0f;
  depthCm_ = 0.0f;
  nextRecoveryAttemptMs_ = 0;
  faultCount_ = 0;
  lastFault_ = SampleResult::BusError;

  prepareForSensorAccess();
  beginWire();

  Sample startupSample{};
  Sample calibratedSample{};
  if (recoverSensor(startupSample, false) &&
      calibrateSurfacePressure(calibratedSample) &&
      commitSample(calibratedSample)) {
    healthy_ = true;
    lastFault_ = SampleResult::Ok;
    return;
  }

  healthy_ = false;
  lastFault_ = SampleResult::BadValue;
  nextRecoveryAttemptMs_ = millis() + kRecoveryRetryMs;
}

bool SeaPerchDepthSensor::update() {
  if (!begun_) {
    return false;
  }

  // This happens before every possible sensor access and before every false
  // return, so the calling sketch can never leave an old motor command active.
  stopMotor();

  if (pendingRecoveredSampleValid_) {
    const Sample sample = pendingRecoveredSample_;
    pendingRecoveredSampleValid_ = false;
    if (commitSample(sample)) {
      healthy_ = true;
      lastFault_ = SampleResult::Ok;
      return true;
    }
    markFault(SampleResult::BadValue);
    return false;
  }

  if (awaitingPostResetSample_) {
    prepareForSensorAccess();
    Sample sample{};
    const SampleResult result =
        readFreshSample(sample, kFreshSampleTimeoutMs);
    if (result == SampleResult::Ok && commitSample(sample)) {
      awaitingPostResetSample_ = false;
      healthy_ = true;
      lastFault_ = SampleResult::Ok;
      return true;
    }
    markFault(result == SampleResult::Ok ? SampleResult::BadValue : result);
    return false;
  }

  if (healthy_ && baselineValid_) {
    prepareForSensorAccess();
    Sample sample{};
    const SampleResult result =
        readFreshSample(sample, kFreshSampleTimeoutMs);
    if (result == SampleResult::Ok && commitSample(sample)) {
      return true;
    }
    markFault(result == SampleResult::Ok ? SampleResult::BadValue : result);
    return false;
  }

  if (!timeReached(millis(), nextRecoveryAttemptMs_)) {
    return false;
  }

  prepareForSensorAccess();
  const bool hadBaseline = baselineValid_;
  const bool allowFastResume =
      hadBaseline && lastFault_ == SampleResult::BusError;
  Sample recoveredSample{};
  if (!recoverSensor(recoveredSample, allowFastResume)) {
    nextRecoveryAttemptMs_ = millis() + kRecoveryRetryMs;
    return false;
  }

  if (!baselineValid_) {
    Sample calibratedSample{};
    if (!calibrateSurfacePressure(calibratedSample)) {
      Wire.end();
      healthy_ = false;
      lastFault_ = SampleResult::BadValue;
      nextRecoveryAttemptMs_ = millis() + kRecoveryRetryMs;
      return false;
    }

    // Sixteen calibration samples have already validated the data stream.
    pendingRecoveredSample_ = calibratedSample;
    pendingRecoveredSampleValid_ = true;
    healthy_ = true;
    lastFault_ = SampleResult::Ok;
    return false;
  }

  lastFault_ = SampleResult::Ok;
  if (lastRecoveryUsedFastResume_) {
    // This sample belongs to the existing stream and is safe to reuse once.
    pendingRecoveredSample_ = recoveredSample;
    pendingRecoveredSampleValid_ = true;
    healthy_ = true;
  } else {
    // Do not drive from the first sample following SWRESET. Require one more
    // fresh sample on the next update before control is allowed to continue.
    awaitingPostResetSample_ = true;
    healthy_ = false;
  }
  return false;
}

bool SeaPerchDepthSensor::ready() const {
  return begun_ && healthy_ && baselineValid_ &&
         !awaitingPostResetSample_ && !pendingRecoveredSampleValid_;
}

float SeaPerchDepthSensor::depthCm() const { return depthCm_; }

float SeaPerchDepthSensor::pressureHpa() const { return pressureHpa_; }

float SeaPerchDepthSensor::temperatureC() const { return temperatureC_; }

float SeaPerchDepthSensor::temperatureF() const {
  return temperatureC_ * 9.0f / 5.0f + 32.0f;
}

uint32_t SeaPerchDepthSensor::faultCount() const { return faultCount_; }

void SeaPerchDepthSensor::stopMotor() const {
  if (motorOffCallback_ != nullptr) {
    motorOffCallback_();
  }
}

void SeaPerchDepthSensor::prepareForSensorAccess() const {
  stopMotor();
  delay(kMotorI2cQuietTimeMs);
}

void SeaPerchDepthSensor::beginWire() {
  Wire.begin();
  Wire.setClock(kI2cClockHz);
  Wire.setWireTimeout(kI2cTimeoutUs, false);
}

bool SeaPerchDepthSensor::timeReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

void SeaPerchDepthSensor::driveLineLow(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void SeaPerchDepthSensor::releaseLine(uint8_t pin) {
  pinMode(pin, INPUT_PULLUP);
}

bool SeaPerchDepthSensor::waitForLineHigh(uint8_t pin, uint32_t timeoutUs) {
  const uint32_t startedUs = micros();
  while (digitalRead(pin) == LOW) {
    if (static_cast<uint32_t>(micros() - startedUs) >= timeoutUs) {
      return false;
    }
  }
  return true;
}

bool SeaPerchDepthSensor::clearI2cBus() {
  Wire.end();
  releaseLine(SDA);
  releaseLine(SCL);
  delayMicroseconds(5);

  if (!waitForLineHigh(SCL, 1000UL)) {
    releaseLine(SDA);
    releaseLine(SCL);
    return false;
  }

  for (uint8_t pulse = 0;
       pulse < 9 && digitalRead(SDA) == LOW;
       ++pulse) {
    driveLineLow(SCL);
    delayMicroseconds(5);
    releaseLine(SCL);
    if (!waitForLineHigh(SCL, 1000UL)) {
      releaseLine(SDA);
      releaseLine(SCL);
      return false;
    }
    delayMicroseconds(5);
  }

  // Generate STOP without ever actively driving an I2C line HIGH.
  driveLineLow(SDA);
  driveLineLow(SCL);
  delayMicroseconds(5);
  releaseLine(SCL);
  if (!waitForLineHigh(SCL, 1000UL)) {
    releaseLine(SDA);
    releaseLine(SCL);
    return false;
  }
  delayMicroseconds(5);
  releaseLine(SDA);
  delayMicroseconds(5);

  return digitalRead(SCL) == HIGH && digitalRead(SDA) == HIGH;
}

bool SeaPerchDepthSensor::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kSensorAddress);
  if (Wire.write(reg) != 1 || Wire.write(value) != 1) {
    (void)Wire.endTransmission(true);
    return false;
  }
  return Wire.endTransmission(true) == 0;
}

bool SeaPerchDepthSensor::readRegisters(uint8_t firstRegister,
                                        uint8_t *destination,
                                        size_t length) {
  if (destination == nullptr || length == 0) {
    return false;
  }

  Wire.beginTransmission(kSensorAddress);
  if (Wire.write(firstRegister) != 1) {
    (void)Wire.endTransmission(true);
    return false;
  }
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t received =
      Wire.requestFrom(kSensorAddress, length, static_cast<bool>(true));
  if (received != length) {
    while (Wire.available() > 0) {
      (void)Wire.read();
    }
    return false;
  }

  for (size_t index = 0; index < length; ++index) {
    const int value = Wire.read();
    if (value < 0) {
      return false;
    }
    destination[index] = static_cast<uint8_t>(value);
  }
  return true;
}

bool SeaPerchDepthSensor::readRegister(uint8_t reg, uint8_t &value) {
  return readRegisters(reg, &value, 1);
}

bool SeaPerchDepthSensor::writeRegisterChecked(uint8_t reg, uint8_t value,
                                               uint8_t mask) {
  uint8_t readback = 0;
  return writeRegister(reg, value) && readRegister(reg, readback) &&
         (readback & mask) == (value & mask);
}

bool SeaPerchDepthSensor::registerMatches(uint8_t reg, uint8_t expected,
                                          uint8_t mask) {
  uint8_t value = 0;
  return readRegister(reg, value) &&
         (value & mask) == (expected & mask);
}

bool SeaPerchDepthSensor::sensorConfigurationIsValid() {
  uint8_t identity = 0;
  return readRegister(kWhoAmIRegister, identity) &&
         identity == kExpectedWhoAmI &&
         registerMatches(kCtrl1Register, kCtrl1_10HzBdu, 0x7F) &&
         registerMatches(kCtrl2Register, kCtrl2AutoIncrement, 0xFF) &&
         registerMatches(kInterruptConfigRegister, 0x00, 0xFF) &&
         registerMatches(kPressureOffsetLowRegister, 0x00, 0xFF) &&
         registerMatches(kPressureOffsetHighRegister, 0x00, 0xFF);
}

bool SeaPerchDepthSensor::configureSensorBounded() {
  uint8_t value = 0;
  if (!readRegister(kWhoAmIRegister, value) || value != kExpectedWhoAmI) {
    return false;
  }

  if (!writeRegister(kCtrl2Register,
                     kCtrl2AutoIncrement | kCtrl2SoftwareReset)) {
    return false;
  }

  const uint32_t resetStartedMs = millis();
  do {
    delay(2);
    if (!readRegister(kCtrl2Register, value)) {
      return false;
    }
    if ((value & kCtrl2SoftwareReset) == 0) {
      break;
    }
  } while (static_cast<uint32_t>(millis() - resetStartedMs) <
           kSensorResetTimeoutMs);

  if ((value & kCtrl2SoftwareReset) != 0) {
    return false;
  }

  if (!readRegister(kWhoAmIRegister, value) || value != kExpectedWhoAmI) {
    return false;
  }
  if (!writeRegisterChecked(kInterruptConfigRegister, 0x00, 0xFF)) {
    return false;
  }
  if (!writeRegisterChecked(kPressureOffsetLowRegister, 0x00, 0xFF) ||
      !writeRegisterChecked(kPressureOffsetHighRegister, 0x00, 0xFF)) {
    return false;
  }
  if (!writeRegisterChecked(kCtrl2Register, kCtrl2AutoIncrement, 0xFF)) {
    return false;
  }
  return writeRegisterChecked(kCtrl1Register, kCtrl1_10HzBdu, 0x7F);
}

SeaPerchDepthSensor::SampleResult SeaPerchDepthSensor::readSensorSample(
    Sample &sample) {
  uint8_t status = 0;
  if (!readRegister(kStatusRegister, status)) {
    return SampleResult::BusError;
  }
  if ((status & kStatusPressureAndTemperatureReady) !=
      kStatusPressureAndTemperatureReady) {
    return SampleResult::NotReady;
  }

  // With BDU enabled, PRESS_OUT_H must be the final output address read.
  uint8_t temperatureBytes[2] = {0, 0};
  uint8_t pressureBytes[3] = {0, 0, 0};
  if (!readRegisters(kTemperatureOutLRegister, temperatureBytes,
                     sizeof(temperatureBytes)) ||
      !readRegisters(kPressureOutXlRegister, pressureBytes,
                     sizeof(pressureBytes))) {
    return SampleResult::BusError;
  }

  uint32_t rawPressureBits =
      static_cast<uint32_t>(pressureBytes[0]) |
      (static_cast<uint32_t>(pressureBytes[1]) << 8) |
      (static_cast<uint32_t>(pressureBytes[2]) << 16);
  if ((rawPressureBits & 0x00800000UL) != 0) {
    rawPressureBits |= 0xFF000000UL;
  }
  const int32_t rawPressure = static_cast<int32_t>(rawPressureBits);

  const uint16_t rawTemperatureBits =
      static_cast<uint16_t>(temperatureBytes[0]) |
      (static_cast<uint16_t>(temperatureBytes[1]) << 8);
  const int16_t rawTemperature = static_cast<int16_t>(rawTemperatureBits);

  sample.pressureHpa = static_cast<float>(rawPressure) / 4096.0f;
  sample.temperatureC = static_cast<float>(rawTemperature) / 100.0f;

  if (!isfinite(sample.pressureHpa) || !isfinite(sample.temperatureC) ||
      sample.pressureHpa < kMinimumPressureHpa ||
      sample.pressureHpa > kMaximumPressureHpa ||
      sample.temperatureC < kMinimumTemperatureC ||
      sample.temperatureC > kMaximumTemperatureC) {
    return SampleResult::BadValue;
  }
  return SampleResult::Ok;
}

SeaPerchDepthSensor::SampleResult SeaPerchDepthSensor::readFreshSample(
    Sample &sample, uint32_t timeoutMs) {
  const uint32_t startedMs = millis();
  do {
    const SampleResult result = readSensorSample(sample);
    if (result != SampleResult::NotReady) {
      return result;
    }
    delay(kFreshSamplePollMs);
  } while (static_cast<uint32_t>(millis() - startedMs) < timeoutMs);
  return SampleResult::NotReady;
}

bool SeaPerchDepthSensor::recoverSensor(Sample &recoveredSample,
                                        bool allowFastResume) {
  lastRecoveryUsedFastResume_ = false;
  bool tryFastResume = allowFastResume;

  for (uint8_t attempt = 0; attempt < kRecoveryAttemptsPerRun; ++attempt) {
    (void)clearI2cBus();
    beginWire();

    if (tryFastResume) {
      if (sensorConfigurationIsValid() &&
          readFreshSample(recoveredSample, kFreshSampleTimeoutMs) ==
              SampleResult::Ok) {
        lastRecoveryUsedFastResume_ = true;
        return true;
      }
      tryFastResume = false;
    } else if (configureSensorBounded() &&
               readFreshSample(recoveredSample, kFreshSampleTimeoutMs) ==
                   SampleResult::Ok) {
      return true;
    }

    Wire.end();
    if (attempt + 1U < kRecoveryAttemptsPerRun) {
      delay(2);
    }
  }

  beginWire();
  return false;
}

bool SeaPerchDepthSensor::calibrateSurfacePressure(Sample &latestSample) {
  float sumHpa = 0.0f;
  float minimumHpa = kMaximumPressureHpa;
  float maximumHpa = kMinimumPressureHpa;

  for (uint8_t index = 0; index < kBaselineSampleCount; ++index) {
    Sample sample{};
    if (readFreshSample(sample, kFreshSampleTimeoutMs) != SampleResult::Ok) {
      return false;
    }

    latestSample = sample;
    sumHpa += sample.pressureHpa;
    if (sample.pressureHpa < minimumHpa) {
      minimumHpa = sample.pressureHpa;
    }
    if (sample.pressureHpa > maximumHpa) {
      maximumHpa = sample.pressureHpa;
    }
  }

  if ((maximumHpa - minimumHpa) > kMaxBaselineSpreadHpa) {
    return false;
  }

  surfacePressureHpa_ = sumHpa / static_cast<float>(kBaselineSampleCount);
  baselineValid_ = true;
  return true;
}

bool SeaPerchDepthSensor::commitSample(const Sample &sample) {
  if (!baselineValid_ || !isfinite(sample.pressureHpa) ||
      !isfinite(sample.temperatureC)) {
    return false;
  }

  const float newDepthCm =
      (sample.pressureHpa - surfacePressureHpa_) / hpaPerCm_;
  if (!isfinite(newDepthCm)) {
    return false;
  }

  // Publish all fields together only after every value has been validated.
  pressureHpa_ = sample.pressureHpa;
  temperatureC_ = sample.temperatureC;
  depthCm_ = newDepthCm;
  return true;
}

void SeaPerchDepthSensor::markFault(SampleResult reason) {
  stopMotor();
  Wire.end();
  healthy_ = false;
  awaitingPostResetSample_ = false;
  pendingRecoveredSampleValid_ = false;
  lastFault_ = reason;
  ++faultCount_;
  nextRecoveryAttemptMs_ = millis();
}
