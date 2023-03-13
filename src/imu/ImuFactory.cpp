//
// Created by 4c3y (acey) on 04.10.22.
//

#include <imu/ImuFactory.h>
#include <imu/adis16448.h>
#include <log++.h>
#include <memory>

/**
 * Helper class to create an imu interface
 * @param imu_name
 * @param spi_path path to spi device
 * @return uninitialized imu interface on success, otherwise nullptr
 */
ImuInterface *ImuFactory::createImuByName(const std::string &imu_name,
                                          const std::string &spi_path) {
  LOG(I, "Imu type: " << imu_name);
  if (imu_name == "adis16448") {
    auto adis = new Adis16448(spi_path);
    return adis;
  }


  LOG(E, "Imu of type " << imu_name << " not supported");
  return nullptr;
}
