#ifndef MAV_IMU_SRC_IMU_BMI088_H_
#define MAV_IMU_SRC_IMU_BMI088_H_

#include "imu_interface.h"
#include "spi_driver.h"
#include <optional>
#include <string>

class Bmi088 : public ImuInterface {
 public:
  /**
   * Bmi088 Constructor
   * @param path to spidev, e.g., "/dev/spidev0.1".
   */
  explicit Bmi088(std::string acc_path, std::string gyro_path);

  bool selftest() override;
  bool init() override;
  std::optional<vec3<double>> getGyro() override { return std::nullopt; }

  std::optional<vec3<double>> getAcceleration() override { return std::nullopt; }

  /**
   * Free file descriptor
   * @return true if successful, otherwise false and errno is set.
   */
  bool close() final;

  int getRaw(std::vector<byte> cmd) override;

 private:
  SpiDriver acc_spi_driver_;
  SpiDriver gyro_spi_driver_;
};

#endif // MAV_IMU_SRC_IMU_BMI088_H_