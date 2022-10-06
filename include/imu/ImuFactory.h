//
// Created by 4c3y (acey) on 04.10.22.
//

#ifndef MAV_IMU_SRC_IMU_IMUFACTORY_H_
#define MAV_IMU_SRC_IMU_IMUFACTORY_H_

#include <memory>
#include "imu_interface.h"

class ImuFactory {
 public:
  static ImuInterface* createImuByName(const std::string &imu_name, const std::string &spi_path);
};

#endif //MAV_IMU_SRC_IMU_IMUFACTORY_H_
