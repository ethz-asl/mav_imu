//
// Created by acey on 25.08.22.
//

#include "log++/log++.h"
#include "imu/adis16448.h"
#include "adis16448_cmds.h"
#include <iostream>
#include <linux/spi/spidev.h>
#include <cstring>


Adis16448::Adis16448(const std::string& path) : spi_driver_(path) {}

bool Adis16448::init() {

  if (!spi_driver_.open()) {
    std::cout << "open failed: " << strerror(errno) << std::endl;
    return false;
  }

  if (!spi_driver_.setMode(SPI_MODE_3)) {
    std::cout << "Setmode failed" << std::endl;
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
    LOG(E, "self check failed");
    return false;
  }

  LOG(I, "Self check passed");
  return true;
}

bool Adis16448::burstread() {
  std::vector<byte> req = CMD(DIAG_STAT);
  req.resize(req.size() + 12);

  std::vector<byte> res;

  res = spi_driver_.burst(req, req.size() + 12);

  for (byte b: res) {
    std::cout << b << " ";
  }
  std::cout << std::endl;

  return false;
}

bool Adis16448::close() {
  return spi_driver_.close();
}

vec3<int> Adis16448::getGyro() {
  vec3<int> gyro{};

  gyro.x = unsignedWordToInt(spi_driver_.xfer({XGYRO_OUT, 0x00}));
  gyro.y = unsignedWordToInt(spi_driver_.xfer({YGYRO_OUT, 0x00}));
  gyro.z = unsignedWordToInt(spi_driver_.xfer({ZGYRO_OUT, 0x00}));

  return gyro;
}

vec3<double> Adis16448::getAcceleration() {
  //twos complement format, 1200 LSB/g, 0 g = 0x0000
  vec3<double> acceleration{};

  acceleration.x = signedWordToInt(spi_driver_.xfer({XACCL_OUT, 0x00}));
  acceleration.y = signedWordToInt(spi_driver_.xfer({YACCL_OUT, 0x00}));
  acceleration.z = signedWordToInt( spi_driver_.xfer({ZACCL_OUT, 0x00}));

  acceleration / 1200.;
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
  //Twos complement, 0.07386°C/LSB, 31°C = 0x0000
  int a = signedWordToInt(spi_driver_.xfer({TEMP_OUT, 0x00}));
  return 31 + (a * 0.07386);
}

int Adis16448::getRaw(std::vector<byte> cmd) {
  std::vector<byte> res = spi_driver_.xfer(cmd);
  return unsignedWordToInt(res);
}

int Adis16448::unsignedWordToInt(const std::vector<byte>& word) {
  return ((word[0] << 8) + word[1]);
}

int Adis16448::signedWordToInt(const std::vector<byte> &word) {
  //byte is unsigned char, cast to signed char
  char byte1 = (char) word[0];
  char byte2 = (char) word[1];

  return (byte1 << 8) + byte2;
}
