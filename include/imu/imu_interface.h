//
// Created by acey on 24.08.22.
//

#ifndef MAV_IMU_INCLUDE_IMU_INTERFACE_H_
#define MAV_IMU_INCLUDE_IMU_INTERFACE_H_

template<typename T>
struct vec3 {
  T x;
  T y;
  T z;
};


class ImuInterface {
 public:
  virtual bool selftest() = 0;
  virtual bool init() = 0;
  virtual bool close() = 0;
/*
  //! gyroscope output
  virtual vec3<T> getGyroscope() = 0;
  //! gyroscope bias offset factor
  virtual vec3<T> getGyroscopeOffset() = 0;

  //! accelerometer output
  virtual vec3<T> getAccelerometer() = 0;
  //! acceleration bias offset factor
  virtual vec3<T> getAccelerometerOffset() = 0;

  //! magnetometer measurement
  virtual vec3<T> getMagnetometer() = 0;
  //! magnetometer hard iron factor
  virtual vec3<T> getMagnetometerHic() = 0;
  //! magnetometer soft iron factor
  virtual vec3<T> getMagnetometerSic() = 0;

  virtual int getBarometer() = 0;
  virtual int getTemperature() = 0;

  virtual int getSerialnumber();
*/
};

#endif //MAV_IMU_INCLUDE_IMU_INTERFACE_H_
