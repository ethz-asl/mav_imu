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

  auto rslt = bmi08xa_init(&dev_);
  printErrorCodeResults("bmi08xa_init", rslt);

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

void Bmi088::printErrorCodeResults(const std::string &api_name, int8_t rslt) {
  if (rslt != BMI08_OK) {
    LOG(E, api_name.c_str() << "\t");
    if (rslt == BMI08_E_NULL_PTR) {
      LOG(E, "Error [" << rslt << "] : Null pointer\r\n");
    } else if (rslt == BMI08_E_COM_FAIL) {
      LOG(E, "Error [" << rslt << "] : Communication failure\r\n");
    } else if (rslt == BMI08_E_DEV_NOT_FOUND) {
      LOG(E, "Error [" << rslt << "] : Device not found\r\n");
    } else if (rslt == BMI08_E_OUT_OF_RANGE) {
      LOG(E, "Error [" << rslt << "] : Out of Range\r\n");
    } else if (rslt == BMI08_E_INVALID_INPUT) {
      LOG(E, "Error [" << rslt << "] : Invalid input\r\n");
    } else if (rslt == BMI08_E_CONFIG_STREAM_ERROR) {
      LOG(E, "Error [" << rslt << "] : Config stream error\r\n");
    } else if (rslt == BMI08_E_RD_WR_LENGTH_INVALID) {
      LOG(E, "Error [" << rslt << "] : Invalid Read write length\r\n");
    } else if (rslt == BMI08_E_INVALID_CONFIG) {
      LOG(E, "Error [" << rslt << "] : Invalid config\r\n");
    } else if (rslt == BMI08_E_FEATURE_NOT_SUPPORTED) {
      LOG(E, "Error [" << rslt << "] : Feature not supported\r\n");
    } else if (rslt == BMI08_W_FIFO_EMPTY) {
      printf("Warning [%d] : FIFO empty\r\n");
    } else {
      LOG(E, "Error [" << rslt << "] : Unknown error code\r\n");
    }
  }
}