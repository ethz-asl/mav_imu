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
  std::vector<byte> res;
  res = spi_driver_.xfer(CMD(DIAG_STAT));

  if (res.empty()) {
    return false;
  }

  if ((res[1] << 8) + res[0] ^ 0x00) {
    std::cout << "self check failed" << std::endl;
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

