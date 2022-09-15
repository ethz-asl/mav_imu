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

  bool selftest() override;
  bool init() override;
  vec3<double> getGyro() override;
  vec3<double> getAcceleration() override;
  vec3<double> getMagnetometer() override;
  double getBarometer() override;

  /**
   * Note that this temperature represents
   * an internal temperature reading, which does not precisely
   * represent external conditions. The intended use of TEMP_OUT
   * is to monitor relative changes in temperature.
   */
  double getTemperature() override;
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
  //!Convert spi output to measurement unit required by the ImuInterface

 /**
  * @param gyro
  * @return rad/s
  */
  static vec3<double> convertGyro(vec3<double> gyro);

  /**
   * @param accel
   * @return m/s^2
   */
  static vec3<double> convertAcceleration(vec3<double> accel);

  /**
   * @param magnetometer
   * @return tesla [T]
   */
  static vec3<double> convertMagnetometer(vec3<double> magnetometer);
  SpiDriver spi_driver_;
};

#endif //MAV_IMU_SRC_IMU_ADIS16448_H_
