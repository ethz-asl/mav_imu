//
// Created by 4c3y (acey) on 04.10.22.
//

#include <imu/ImuFactory.h>
#include <imu/adis16448.h>
#include <imu/bmi088.h>
#include <log++.h>
#include <memory>

/**
 * Helper class to create and initialize imu interface
 * @param imu_name
 * @param spi_path path to spi device
 * @return initialized imu interface on success, otherwise nullptr
 */
ImuInterfacePtr ImuFactory::createImuByName(const std::string &imu_name,
                                          const std::string &spi_path) {
  LOG(I, "Imu type: " << imu_name);
  if (imu_name == "adis16448") {

    auto adis = std::make_shared<Adis16448>(spi_path);

    if (adis->init() && adis->setBurstCRCEnabled(true)) { return adis; }
    LOG(E, "Failed to initialize " << imu_name);
    return nullptr;
  } else if (imu_name == "bmi088") {
    // Select the second Chip Select for the gyro.
    // get the last letter of the string
    char last_letter = spi_path[spi_path.length() - 1];
    std::string gyro_path;

    if (last_letter == '0') {
      // create the new string with last letter one
      gyro_path = spi_path.substr(0, spi_path.length() - 1) + '1';
    } else {
      // create the new string with last letter zero
      gyro_path = spi_path.substr(0, spi_path.length() - 1) + '0';
    }
    LOG(I,
        "Opening BMI088 with accelerometer CS \""
            << spi_path.c_str() << "\" and gyroscope CS \"" << gyro_path.c_str()
            << "\".");
    auto bmi = std::make_shared<Bmi088>(spi_path, gyro_path);

    if (bmi->init()) { return bmi; }
    LOG(E, "Failed to initialize " << imu_name);
    return nullptr;
  }

  LOG(E, "Imu of type " << imu_name << " not supported");
  return nullptr;
}
