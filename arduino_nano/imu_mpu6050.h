// MPU6050 IMU on I2C (address 0x68, AD0 to GND). Reads raw accelerometer and gyro counts; the Pi
// converts them to m/s^2 and rad/s.
#pragma once

#include <Arduino.h>
#include <Wire.h>

class Mpu6050 {
 public:
  // dlpf: digital low-pass filter setting 0..6. sample_divider: internal rate 1 kHz/(1+divider).
  Mpu6050(uint8_t dlpf, uint8_t sample_divider) : dlpf_(dlpf), divider_(sample_divider) {}

  // Wakes the sensor and configures it. Returns false if no MPU6050 answers.
  bool begin() {
    ok_ = read_register(0x75) == 0x68  // WHO_AM_I
          && write_register(0x6B, 1)    // PWR_MGMT_1: wake up, clock from the gyro
          && write_register(0x1A, dlpf_)     // CONFIG: low-pass filter
          && write_register(0x19, divider_)  // SMPLRT_DIV
          && write_register(0x1B, 0)    // GYRO_CONFIG: +-250 deg/s
          && write_register(0x1C, 0);   // ACCEL_CONFIG: +-2 g
    return ok_;
  }

  bool ok() const { return ok_; }

  // Reads one sample (raw counts). Returns false on an I2C error.
  bool read(int16_t accel[3], int16_t gyro[3]) {
    if (!ok_) return false;
    uint8_t b[14];  // accel x,y,z | temperature | gyro x,y,z (big endian)
    Wire.beginTransmission(ADDRESS);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(ADDRESS, (uint8_t)14) != 14) {
      ok_ = false;  // the main loop will call begin() again
      return false;
    }
    for (uint8_t i = 0; i < 14; ++i) b[i] = Wire.read();
    for (uint8_t i = 0; i < 3; ++i) {
      accel[i] = int16_t((b[2 * i] << 8) | b[2 * i + 1]);
      gyro[i] = int16_t((b[8 + 2 * i] << 8) | b[8 + 2 * i + 1]);
    }
    return true;
  }

 private:
  static const uint8_t ADDRESS = 0x68;

  bool write_register(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }
  int read_register(uint8_t reg) {
    Wire.beginTransmission(ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(ADDRESS, (uint8_t)1) != 1) return -1;
    return Wire.read();
  }

  uint8_t dlpf_, divider_;
  bool ok_ = false;
};
