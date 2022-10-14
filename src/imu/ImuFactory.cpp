//
// Created by 4c3y (acey) on 04.10.22.
//

#include <imu/ImuFactory.h>
#include <imu/adis16448.h>
#include <log++.h>
#include <memory>

/**
 * Helper class to create and initialize imu interface
 * @param imu_name
 * @param spi_path path to spi device
 * @return initialized imu interface
 */
ImuInterface *ImuFactory::createImuByName(const std::string &imu_name,
                                          const std::string &spi_path) {
  if (imu_name == "adis16448") {
    auto adis = new Adis16448(spi_path);

    adis->init();
    adis->setBurstCRCEnabled(true);
    return adis;
  }

  LOG(F, "No imu with name " << imu_name << "found");
  abort();
}
