#include "imu/bmi088.h"

#include <bmi08x.h>
#include <iostream>
#include <linux/spi/spidev.h>
#include <log++.h>
#include <string>

Bmi088::Bmi088(std::string acc_path, std::string gyro_path)
    : acc_spi_driver_(std::move(acc_path)),
      gyro_spi_driver_(std::move(gyro_path)) {
  dev_.intf_ptr_accel = &acc_spi_driver_;
  dev_.intf_ptr_gyro  = &gyro_spi_driver_;
  dev_.intf           = BMI08_SPI_INTF;
  dev_.variant        = BMI088_VARIANT;
  dev_.read           = &(Bmi088::readReg);
  dev_.write          = &(Bmi088::writeReg);
  dev_.delay_us       = &(Bmi088::usSleep);
}

bool Bmi088::selftest() {
  LOG(E, "Bmi088 selftest not implemented.");
  return false;
}

bool Bmi088::init() {
  if (!acc_spi_driver_.open()) {
    LOG(E, "Accelerometer open failed: " << strerror(errno));
    return false;
  }

  if (!gyro_spi_driver_.open()) {
    LOG(E, "Gyroscope open failed: " << strerror(errno));
    return false;
  }

  if (!acc_spi_driver_.setMode(SPI_MODE_3)) {
    LOG(E, "Accelerometer setmode failed");
    return false;
  }

  if (!gyro_spi_driver_.setMode(SPI_MODE_3)) {
    LOG(E, "Gyroscope setmode failed");
    return false;
  }

  //int8_t rslt;
  /*initialize bmi088 accel sensor*/
  //rslt = bmi08a_init(&dev_);

  return true;
}

bool Bmi088::close() {
  return acc_spi_driver_.close() && gyro_spi_driver_.close();
}

int Bmi088::getRaw(std::vector<byte> cmd) {
  LOG(E, "Bmi088 getRaw not implemented.");
}

int8_t Bmi088::readReg(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr) {
  reg_addr |= 0x80;
  auto res = static_cast<SpiDriver *>(intf_ptr)->xfer({reg_addr}, len, 2000000);
  reg_data = res.data();
  return res.empty() ? -1 : 0;
}

int8_t Bmi088::writeReg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr) {
  reg_addr &= 0x7f;
  auto res = static_cast<SpiDriver *>(intf_ptr)->xfer({reg_addr}, len, 2000000);
  reg_data = res.data();
  return res.empty() ? -1 : 0;
}

void Bmi088::usSleep(uint32_t period, void *intf_ptr) { usleep(period); }