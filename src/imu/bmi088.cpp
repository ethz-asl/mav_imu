#include "imu/bmi088.h"

#include <log++.h>

Bmi088::Bmi088(const std::string &path) : spi_driver_(path) {}

bool Bmi088::selftest() {
  LOG(E, "Bmi088 selftest not implemented.");
  return false;
}

bool Bmi088::init() {
  LOG(E, "Bmi088 interface not implemented.");
  return false;
}

bool Bmi088::close() {
  LOG(E, "Bmi088 close not implemented.");
  return false;
}

int Bmi088::getRaw(std::vector<byte> cmd) {
  LOG(E, "Bmi088 getRaw not implemented.");
}