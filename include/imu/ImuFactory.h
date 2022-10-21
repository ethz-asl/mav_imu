//
// Created by 4c3y (acey) on 04.10.22.
//

#ifndef MAV_IMU_SRC_IMU_IMUFACTORY_H_
#define MAV_IMU_SRC_IMU_IMUFACTORY_H_

#include "imu_interface.h"
#include <memory>

class ImuFactory {
 public:
  static ImuInterface *createImuByName(const std::string &imu_name, const std::string &spi_path);
};

#endif //MAV_IMU_SRC_IMU_IMUFACTORY_H_
