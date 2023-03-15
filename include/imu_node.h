//
// Created by acey on 24.08.22.
//

#ifndef MAV_IMU__IMU_TEST_H_
#define MAV_IMU__IMU_TEST_H_

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Empty.h>

#include "imu/adis16448.h"
#include "spi_driver.h"

class ImuNode {
 public:
  ImuNode(ImuInterface &imu_interface, int frequency);
  bool init();
  int run();

  inline static bool run_node = true;

 private:
  void processImuData();
  void processMagneticFieldData();
  void processTemperature();
  void processFluidpressure();

  bool runGyroCalibration(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  ros::Publisher imu_data_raw_pub_{};
  ros::Publisher imu_mag_pub_{};
  ros::Publisher imu_temp_pub_{};
  ros::Publisher imu_baro_pub_{};

  ros::ServiceServer gyro_calib_srv_{};

  ImuInterface &imu_interface_;
  int frequency_{};

  ros::Time time_now_{};
  struct ImuBurstResult imu_burst_result_ {};
};

#endif // MAV_IMU__IMU_TEST_H_
