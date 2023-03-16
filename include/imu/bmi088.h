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
  std::optional<vec3<double>> getGyro() override;

  std::optional<vec3<double>> getAcceleration() override;

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

  /*!
  *  @brief Sleep ms function for BMI088 to be passed to BMI device driver.
  *
  *  @param[in] period   : Sleep time in micro seconds.
  *  @param[in] intf_ptr : Void pointer that can enable the linking of descriptors for interface related callbacks.
  *
  *  @return void.
  */
  static void usSleep(uint32_t period, void *intf_ptr);

  /*!
  *  @brief Prints the execution status of the BMI APIs.
  *
  *  @param[in] api_name : Name of the API whose execution status has to be printed.
  *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
  *
  *  @return void.
  */
  void printErrorCodeResults(const std::string &api_name, int8_t rslt);

  /*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 *
 * @param val: 16-bit accelerometer register value.
 * @param g_range: Accelerometer range, e.g., BMI088_ACCEL_RANGE_24G.
 * @param bit_width
 *
 * @return Acceleration in m/s
 */
  static float lsbToMps2(int16_t val, int8_t g_range, uint8_t bit_width);

  /*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 *
 * @param val: 16-bit gyroscope register value.
 * @param g_range: Gyro range, e.g., BMI08_GYRO_RANGE_2000_DPS.
 * @param bit_width
 *
 * @return Angular velocity in DPS
 */
  static float lsbToDps(int16_t val, uint8_t dps_range, uint8_t bit_width);

  SpiDriver acc_spi_driver_;
  SpiDriver gyro_spi_driver_;
  bmi08_dev dev_;

  inline static const constexpr uint32_t spi_transfer_speed_hz_ = 10000000;
};

#endif // MAV_IMU_SRC_IMU_BMI088_H_