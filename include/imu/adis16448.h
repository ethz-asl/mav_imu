//
// Created by acey on 25.08.22.
//

#ifndef MAV_IMU_SRC_IMU_ADIS16448_H_
#define MAV_IMU_SRC_IMU_ADIS16448_H_
#include "imu_interface.h"
#include <string>
#include "spi_driver.h"

class Adis16448 : public ImuInterface {
 public:
  /**
   * Adis16448 Constructor
   * @param path to spidev e.g. /dev/spidev0.1
   */
  explicit Adis16448(const std::string &path);

  /**
   * Imu health check.
   * @return true if successful, otherwise false
   */
  bool selftest() override;
  bool init() override;
  bool burstread();
  vec3<int> getGyro() override;

  /**
   * Gets acceleration data vector
   *
   * @return acceleration in m/s² as double
   */
  vec3<double> getAcceleration() override;
  vec3<int> getMagnetometer() override;

  /**
   * Gets barometric pressure
   *
   * @return QFE pressure in hPa
   */
  double getBarometer() override;

  /**
   * Gets temperature
   *
   * Note that this temperature represents
   * an internal temperature reading, which does not precisely
   * represent external conditions. The intended use of TEMP_OUT
   * is to monitor relative changes in temperature.
   *
   * @return temperature in °C
   */
  double getTemperature() override;

  /**
   * Generic function to read spi register
   * @param cmd
   * @return
   */
  int getRaw(std::vector<byte> cmd) override;

  /**
   * Custom burst mode
   * @return struct with all values.
   */
  ImuBurstResult burst() override;

  /**
   * Free file descriptor
   * @return true if successful, otherwise false and errno is set.
   */
  bool close() final;

  static int signedWordToInt(const std::vector<byte> &word);
  static int unsignedWordToInt(const std::vector<byte> &word);
 private:
  SpiDriver spi_driver_;
};

#endif //MAV_IMU_SRC_IMU_ADIS16448_H_
