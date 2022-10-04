//
// Created by acey on 25.08.22.
//


#include "imu/adis16448.h"
#include "imu/adis16448_cmds.h"
#include <iostream>
#include <linux/spi/spidev.h>
#include <cstring>
#include <log++.h>

Adis16448::Adis16448(const std::string &path) : spi_driver_(path) {}

bool Adis16448::init() {

  if (!spi_driver_.open()) {
    LOG(E, "open failed: " << strerror(errno));
    return false;
  }

  if (!spi_driver_.setMode(SPI_MODE_3)) {
    LOG(E, "Setmode failed");
    return false;
  }

  resetRegisters();
  return true;
}

bool Adis16448::selftest() {
  std::vector<byte> res = spi_driver_.xfer(CMD(DIAG_STAT));

  if (res.empty()) {
    return false;
  }

  if ((res[1] << 8) + res[0] ^ 0x00) {
    //TODO evaluate response
    LOG(E, "Imu self-check failed");
    return false;
  }

  LOG(I, "Adis16448 self-check passed");
  return true;
}

bool Adis16448::close() {
  return spi_driver_.close();
}

vec3<double> Adis16448::getGyro() {
  // twos complement format, 25 LSB/°/sec, 0°/sec = 0x0000
  vec3<double> gyro{};

  gyro.x = signedWordToInt(spi_driver_.xfer({XGYRO_OUT, 0x00}));
  gyro.y = signedWordToInt(spi_driver_.xfer({YGYRO_OUT, 0x00}));
  gyro.z = signedWordToInt(spi_driver_.xfer({ZGYRO_OUT, 0x00}));

  return convertGyro(gyro);
}

vec3<double> Adis16448::convertGyro(vec3<double> gyro) {
  gyro /= 25.; // convert to degrees
  return gyro * (M_PI / 180.); // Convert to rad/s and return
}

vec3<double> Adis16448::getAcceleration() {
  //twos complement format, 1200 LSB/g, 0 g = 0x0000
  vec3<double> acceleration{};

  acceleration.x = signedWordToInt(spi_driver_.xfer({XACCL_OUT, 0x00}));
  acceleration.y = signedWordToInt(spi_driver_.xfer({YACCL_OUT, 0x00}));
  acceleration.z = signedWordToInt(spi_driver_.xfer({ZACCL_OUT, 0x00}));

  return convertAcceleration(acceleration);
}

vec3<double> Adis16448::convertAcceleration(vec3<double> accel) {
  accel /= 1200.; //Convert to g
  return accel * 9.80665;
}

vec3<double> Adis16448::getMagnetometer() {
  //twos complement, 7 LSB/mgauss, 0x0000 = 0 mgauss
  vec3<double> magnetometer{};

  magnetometer.x = signedWordToInt(spi_driver_.xfer({XMAGN_OUT, 0x00}));
  magnetometer.y = signedWordToInt(spi_driver_.xfer({YMAGN_OUT, 0x00}));
  magnetometer.z = signedWordToInt(spi_driver_.xfer({ZMAGN_OUT, 0x00}));

  return convertMagnetometer(magnetometer);
}

vec3<double> Adis16448::convertMagnetometer(vec3<double> magnetometer) {
  magnetometer /= 7.; //Convert to mG;
  magnetometer /= 10000000.; // Convert to tesla
  return magnetometer;
}

double Adis16448::getBarometer() {
  //20 μbar per LSB, 0x0000 = 0 mbar
  int res = unsignedWordToInt(spi_driver_.xfer({BARO_OUT, 0x00}));
  return res * 0.02;
}

double Adis16448::convertBarometer(const std::vector<byte>& word) {
  return unsignedWordToInt(word) * 0.02;
}

double Adis16448::getTemperature() {
  //Twos complement, 0.07386°C/LSB, 31°C = 0x0000, 12bit
  int a = signedWordToInt(spi_driver_.xfer({TEMP_OUT, 0x00}));
  return 31 + (a * 0.07386);
}

double Adis16448::convertTemperature(const std::vector<byte>& word) {
  return 31 + (signedWordToInt(word) * 0.07386);
}

int Adis16448::getRaw(std::vector<byte> cmd) {
  std::vector<byte> res = spi_driver_.xfer(cmd);
  return unsignedWordToInt(res);
}

int Adis16448::unsignedWordToInt(const std::vector<byte> &word) {
  return ((word[0] << CHAR_BIT) + word[1]);
}

int Adis16448::signedWordToInt(const std::vector<byte> &word) {
  return (((int) *(signed char *) (word.data())) * 1 << CHAR_BIT) | word[1];
}

ImuBurstResult Adis16448::burst() {
  std::vector<std::vector<byte>> res = spi_driver_.burst(
      {
          {XGYRO_OUT, 0x00},
          {YGYRO_OUT, 0x00},
          {ZGYRO_OUT, 0x00},
          {XACCL_OUT, 0x00},
          {YACCL_OUT, 0x00},
          {ZACCL_OUT, 0x00},
          {XMAGN_OUT, 0x00},
          {YMAGN_OUT, 0x00},
          {ZMAGN_OUT, 0x00},
          {BARO_OUT, 0x00},
          {TEMP_OUT, 0x00}
      });

  vec3<double> gyro_raw{};
  gyro_raw.x = signedWordToInt(res[0]);
  gyro_raw.y = signedWordToInt(res[1]);
  gyro_raw.z = signedWordToInt(res[2]);

  vec3<double> raw_accel{};
  raw_accel.x = (double) signedWordToInt(res[3]);
  raw_accel.y = (double) signedWordToInt(res[4]);
  raw_accel.z = (double) signedWordToInt(res[5]);

  vec3<double> raw_magn{};
  raw_magn.x = signedWordToInt(res[6]);
  raw_magn.y = signedWordToInt(res[7]);
  raw_magn.z = signedWordToInt(res[8]);

  struct ImuBurstResult ret{};
  ret.gyro = convertGyro(gyro_raw);
  ret.acceleration = convertAcceleration(raw_accel);
  ret.magnetometer = convertMagnetometer(raw_magn);

  ret.baro = convertBarometer(res[9]);
  ret.temp = convertTemperature(res[10]);
  return ret;
}

void Adis16448::resetRegisters() {
  static std::vector<std::vector<byte>> resetRegisters {
      {XGYRO_OFF, 0x0},
      {YGYRO_OFF, 0x0},
      {ZGYRO_OFF, 0x0},
      {XACCL_OFF, 0x0},
      {YACCL_OFF, 0x0},
      {ZACCL_OFF, 0x0},
      {XMAGN_HIC, 0x0},
      {YMAGN_HIC, 0x0},
      {ZMAGN_HIC, 0x0},
      {XMAGN_SIC, 0x0},
      {YMAGN_SIC, 0x0},
      {ZMAGN_SIC, 0x0},
      {MSC_CTRL, 0x00, 0x06},
      {SMPL_PRD, 0x00, 0x01},
      {SENS_AVG, 0x04, 0x02},
      {ALM_MAG1, 0x0},
      {ALM_MAG2, 0x0},
      {ALM_SMPL1, 0x0},
      {ALM_SMPL2, 0x0},
      {ALM_CTRL, 0x0},
  };

  for (const auto& regWrite : resetRegisters) {
    spi_driver_.xfer(regWrite);
  }
}
