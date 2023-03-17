#ifndef MAV_IMU_SRC_IMU_BMI088_H_
#define MAV_IMU_SRC_IMU_BMI088_H_

#include "imu_interface.h"
#include "spi_driver.h"
#include <bmi08_defs.h>
#include <bmi08x.h>
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

  bool init() override;
  std::optional<vec3<double>> getGyro() override;

  std::optional<vec3<double>> getAcceleration() override;

  /**
   * Free file descriptor
   * @return true if successful, otherwise false and errno is set.
   */
  bool close() final;

  int getRaw(std::vector<byte> cmd) override;

  /*!
  *  @brief Reads accelerometer and gyroscope config from registers and prints them out.

  *  @return void.
  */
  void printImuConfig() override;

  /**
   * Custom burst mode
   * @return struct with all values.
   */
  ImuBurstResult burst() override;

 private:
  /*!
  *  @brief Calls the device self test function. A reset should be performed afterwards.

  *  @return Success of selftest.
  */
  bool selftest();

  /*!
  *  @brief Sets up the SPI communication with BMI after a reset.

  *  @return Successful communication test.
  */
  bool setupBmiSpi();

  // Read function for BMI088 to be passed to BMI device driver.
  static int8_t readReg(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

  // Write function for BMI088 to be passed to BMI device driver.
  static int8_t writeReg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

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
  *  @brief Prints the gyro bandwidth setting.

  *  @return void.
  */
  void printGyroBw();

  /*!
  *  @brief Prints the gyro output data rate setting.

  *  @return void.
  */
  void printGyroOdr();

  /*!
  *  @brief Computes the accelerometer range from settings.

  *  @return Accelerometer range in m/s^2.
  */
  static uint8_t computeAccRange(uint8_t accel_cfg_range);

  /*!
  *  @brief Computes the accelerometer bandwidth from settings.

  *  @return Accelerometer bandwidth in Hz.
  */
  static uint8_t computeAccBw(uint8_t accel_cfg_bw);

  /*!
  *  @brief Computes the accelerometer output data rate (ODR) from settings.

  *  @return Accelerometer output data rate in Hz.
  */
  static uint16_t computeAccOdr(uint16_t accel_cfg_odr);

  /*!
  *  @brief Computes the gyroscope range from settings.

  *  @return Gyroscope range in dps.
  */
  static uint16_t computeGyroRange(uint16_t gyro_cfg_range);

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
 * @return Angular velocity in radians per second (RPS)
 */
  static float lsbToRps(int16_t val, uint8_t dps_range, uint8_t bit_width);

  SpiDriver acc_spi_driver_;
  SpiDriver gyro_spi_driver_;
  /*!
  *  @brief BMI device with communication settings and IMU configuration. The configuration will be overwritten in initialization method.
  */
  bmi08_dev dev_{// Communication.
                 .intf_ptr_accel = &acc_spi_driver_,
                 .intf_ptr_gyro  = &gyro_spi_driver_,
                 .intf           = BMI08_SPI_INTF,
                 .variant        = BMI088_VARIANT,
                 .accel_cfg      = bmi08_cfg{.power = BMI08_ACCEL_PM_ACTIVE,
                                             .range = BMI088_ACCEL_RANGE_24G,
                                             .bw    = BMI08_ACCEL_BW_NORMAL,
                                             .odr   = BMI08_ACCEL_ODR_1600_HZ},
                 .gyro_cfg       = bmi08_cfg{.power = BMI08_GYRO_PM_NORMAL,
                                             .range = BMI08_GYRO_RANGE_2000_DPS,
                                             .bw    = BMI08_GYRO_BW_532_ODR_2000_HZ,
                                             .odr   = BMI08_GYRO_BW_532_ODR_2000_HZ},
                 .read_write_len = 32,
                 .read           = &(Bmi088::readReg),
                 .write          = &(Bmi088::writeReg),
                 .delay_us       = &(Bmi088::usSleep)};

  inline static const constexpr uint32_t spi_transfer_speed_hz_ = 10000000;
  inline static const constexpr uint8_t g_range_min_            = 3;
  inline static const constexpr uint16_t dps_range_max_         = 2000;
  inline static const constexpr double acc_odr_min_             = 12.5;
  inline static const constexpr uint8_t acc_bw_osr_max_         = 4;
  inline static const constexpr uint16_t gyro_odr_max_          = 2000;
  inline static const constexpr uint16_t gyro_bw_max_           = 532;
};

#endif // MAV_IMU_SRC_IMU_BMI088_H_