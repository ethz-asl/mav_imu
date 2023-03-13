#ifndef MAV_IMU_SRC_IMU_BMI088_H_
#define MAV_IMU_SRC_IMU_BMI088_H_

#include "imu_interface.h"
#include "spi_driver.h"
#include <bmi08_defs.h>
#include <optional>
#include <string>

class Bmi088 : public ImuInterface {
 public:
  /**
   * Bmi088 Constructor
   * @param path to accelerometer spidev, e.g., "/dev/spidev0.0".
   * @param path to gyro spidev, e.g., "/dev/spidev0.1".
   */
  explicit Bmi088(std::string acc_path, std::string gyro_path);

  bool selftest() override;
  bool init() override;
  std::optional<vec3<double>> getGyro() override { return std::nullopt; }

  std::optional<vec3<double>> getAcceleration() override {
    return std::nullopt;
  }

  /**
   * Free file descriptor
   * @return true if successful, otherwise false and errno is set.
   */
  bool close() final;

  int getRaw(std::vector<byte> cmd) override;

 private:
  // Read function for BMI088 to be passed to BMI device driver.
  static int8_t readReg(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                        void *intf_ptr);

  // Write function for BMI088 to be passed to BMI device driver.
  static int8_t writeReg(uint8_t reg_addr, const uint8_t *reg_data,
                         uint32_t len, void *intf_ptr);

  // Sleep ms function for BMI088 to be passed to BMI device driver.
  static void usSleep(uint32_t period, void *intf_ptr);

  SpiDriver acc_spi_driver_;
  SpiDriver gyro_spi_driver_;
  bmi08_dev dev_;
};

#endif // MAV_IMU_SRC_IMU_BMI088_H_