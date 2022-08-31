//
// Created by acey on 24.08.22.
//

#ifndef MAV_IMU_INCLUDE_IMU_INTERFACE_H_
#define MAV_IMU_INCLUDE_IMU_INTERFACE_H_

#include <sstream>
#include "spi_driver.h"
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


class ImuInterface {
 public:
  virtual bool selftest() = 0;
  virtual bool init() = 0;
  virtual bool close() = 0;

  //! gyroscope output
  virtual vec3<int> getGyro() = 0;
  //! gyroscope bias offset factor
  //virtual vec3<int> getGyroscopeOffset() = 0;

  //! accelerometer output
  virtual vec3<int> getAcceleration() = 0;
  //! acceleration bias offset factor
  //virtual vec3<T> getAccelerometerOffset() = 0;

  //! magnetometer measurement
  virtual vec3<int> getMagnetometer() = 0;
  //! magnetometer hard iron factor
  //virtual vec3<T> getMagnetometerHic() = 0;
  //! magnetometer soft iron factor
  //virtual vec3<T> getMagnetometerSic() = 0;

  virtual double getBarometer() = 0;
  virtual double getTemperature() = 0;

  virtual int getRaw(std::vector<byte> cmd) = 0;

  //virtual int getSerialnumber();
};

#endif //MAV_IMU_INCLUDE_IMU_INTERFACE_H_
