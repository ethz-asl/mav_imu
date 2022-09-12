//
// Created by acey on 25.08.22.
//


#include "imu/adis16448.h"
#include "adis16448_cmds.h"
#include <iostream>
#include <linux/spi/spidev.h>
#include <cstring>
#include "log++/log++.h"

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

vec3<int> Adis16448::getGyro() {
  vec3<int> gyro{};

  gyro.x = signedWordToInt(spi_driver_.xfer({XGYRO_OUT, 0x00}));
  gyro.y = signedWordToInt(spi_driver_.xfer({YGYRO_OUT, 0x00}));
  gyro.z = signedWordToInt(spi_driver_.xfer({ZGYRO_OUT, 0x00}));

  return gyro;
}

vec3<double> Adis16448::getAcceleration() {
  //twos complement format, 1200 LSB/g, 0 g = 0x0000
  vec3<double> acceleration{};

  acceleration.x = signedWordToInt(spi_driver_.xfer({XACCL_OUT, 0x00}));
  acceleration.y = signedWordToInt(spi_driver_.xfer({YACCL_OUT, 0x00}));
  acceleration.z = signedWordToInt(spi_driver_.xfer({ZACCL_OUT, 0x00}));

  return acceleration;
}

vec3<int> Adis16448::getMagnetometer() {
  vec3<int> magnetometer{};

  magnetometer.x = unsignedWordToInt(spi_driver_.xfer({XMAGN_OUT, 0x00}));
  magnetometer.y = unsignedWordToInt(spi_driver_.xfer({YMAGN_OUT, 0x00}));
  magnetometer.z = unsignedWordToInt(spi_driver_.xfer({ZMAGN_OUT, 0x00}));

  return magnetometer;
}

double Adis16448::getBarometer() {
  //20 μbar per LSB, 0x0000 = 0 mbar
  int res = unsignedWordToInt(spi_driver_.xfer({BARO_OUT, 0x00}));
  return res * 0.02;
}

double Adis16448::getTemperature() {
  //Twos complement, 0.07386°C/LSB, 31°C = 0x0000, 12bit
  int a = signedWordToInt(spi_driver_.xfer({TEMP_OUT, 0x00}));
  return 31 + (a * 0.07386);
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
  std::vector<std::vector<byte>> res = spi_driver_.burst({
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


  struct ImuBurstResult ret{};
  ret.gyro = {signedWordToInt(res[0]), signedWordToInt(res[1]) ,signedWordToInt(res[2])};
  ret.acceleration = {
      (double) signedWordToInt(res[3]),
      (double)signedWordToInt(res[4]),
      (double)signedWordToInt(res[5])
  };
  ret.magnetometer = {
      unsignedWordToInt(res[6]),
      unsignedWordToInt(res[7]),
      unsignedWordToInt(res[8])
  };

  ret.baro = unsignedWordToInt(res[9]) * 0.02;
  ret.temp = 31 + (signedWordToInt(res[10]) * 0.07386);
  return ret;
}
