//
// Created by acey on 25.08.22.
//

#ifndef MAV_IMU_SRC_IMU_ADIS16448_H_
#define MAV_IMU_SRC_IMU_ADIS16448_H_
#include "imu_interface.h"
#include <string>
#include "spi_driver.h"

class Adis16448 : public ImuInterface {
 public:
  explicit Adis16448(const std::string &path);
  bool selftest() override;
  bool init() override;
  bool burstread();
  bool close() override;

 private:
  SpiDriver spi_driver_;
};

#endif //MAV_IMU_SRC_IMU_ADIS16448_H_
