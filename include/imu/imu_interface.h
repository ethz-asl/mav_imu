//
// Created by acey on 24.08.22.
//

#ifndef MAV_IMU_INCLUDE_IMU_INTERFACE_H_
#define MAV_IMU_INCLUDE_IMU_INTERFACE_H_

#include "spi_driver.h"
#include <optional>
#include <sstream>

template<typename T>
struct vec3 {
  T x;
  T y;
  T z;

  std::string toString() {
    std::stringstream ss;
    ss << "X: " << x << " Y: " << y << " Z: " << z;
    return ss.str();
  }
};

template<typename T>
inline vec3<T> operator/(const vec3<T> t, T num) {
  return {t.x / num, t.y / num, t.z / num};
}

template<typename T>
inline vec3<T> operator/=(vec3<T> &t, T num) {
  return {t.x /= num, t.y /= num, t.z /= num};
}

template<typename T>
inline vec3<T> operator*(const vec3<T> t, T num) {
  return {t.x * num, t.y * num, t.z * num};
}

template<typename T>
inline vec3<T> operator*=(vec3<T> &t, T num) {
  return {t.x *= num, t.y *= num, t.z *= num};
}

class ImuBurstResult {
 public:
  std::optional<vec3<double>> gyro{std::nullopt};
  std::optional<vec3<double>> acceleration{std::nullopt};
  std::optional<vec3<double>> magnetometer{std::nullopt};
  std::optional<double> baro{std::nullopt};
  std::optional<double> temp{std::nullopt};
};

class ImuInterface {
 public:
  virtual bool init() = 0;
  /**
   * Imu health check.
   * @return true if successful, otherwise false
   */

  /**
   * Cleanup and close files used by IMU
   * @return
   */
  virtual bool close() = 0;

  /**
   * Gets angular velocity from gyro.
   * @return angular velocity in rad/s
   */
  virtual std::optional<vec3<double>> getGyro() = 0;

  /**
   * Gets acceleration data vector
   *
   * @return acceleration in m/s² as double
   */
  virtual std::optional<vec3<double>> getAcceleration() = 0;

  //! magnetometer measurement
  virtual std::optional<vec3<double>> getMagnetometer() {
    return std::nullopt;
  };

  /**
   * Gets barometric pressure
   * @return QFE pressure in hPa
   */
  virtual std::optional<double> getBarometer() { return std::nullopt; };

  /**
   * Gets temperature
   * @return temperature in °C
   */
  virtual std::optional<double> getTemperature() { return std::nullopt; };

  /**
   * Generic function to read spi register
   * @param cmd
   * @return
   */
  virtual int getRaw(std::vector<byte> cmd) = 0;

  /**
   * Reads all sensor data at once
   * @return struct with sensor data. Returns std::nullopt if hardware does not support specific sensor field.
   */
  virtual ImuBurstResult burst() {
    ImuBurstResult res{};
    res.gyro         = getGyro();
    res.acceleration = getAcceleration();
    res.magnetometer = getMagnetometer();
    res.baro         = getBarometer();
    res.temp         = getTemperature();

    return res;
  }

  // virtual int getSerialnumber();

  /*!
  *  @brief Reads accelerometer and gyroscope config from registers and prints them out.

  *  @return void.
  */
  virtual void printImuConfig() = 0;

 protected:
  inline static const constexpr double g_ =
      9.80665; // TODO(rikba): Try looking this up from WolframAlpha on init.
};

#endif // MAV_IMU_INCLUDE_IMU_INTERFACE_H_
