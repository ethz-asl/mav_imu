//
// Created by 4c3y (acey) on 04.10.22.
//

#include <imu/ImuFactory.h>
#include <imu/adis16448.h>
#include <memory>
#include <log++.h>

ImuInterface* ImuFactory::createImuByName(const std::string &imu_name, const std::string &spi_path) {
  if (imu_name == "adis16448") {
    return new Adis16448(spi_path);
  }

  LOG(F, "No imu with name " << imu_name << "found");
  abort();
}
