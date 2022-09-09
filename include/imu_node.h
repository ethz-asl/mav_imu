//
// Created by acey on 24.08.22.
//

#ifndef MAV_IMU__IMU_TEST_H_
#define MAV_IMU__IMU_TEST_H_

#include <string>
#include "spi_driver.h"
#include "imu/adis16448.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuNode {
 public:
  explicit ImuNode(const std::string &path, ImuInterface& imu_interface);
  bool init();
  int run();

  inline static bool run_node = true;

 private:
  sensor_msgs::Imu processImuData();
  ImuInterface &imu_interface_;
  ros::NodeHandle nh_{};
  ros::Publisher imu_data_raw_pub_{};
  ros::Publisher imu_mag_pub_{};
};

#endif //MAV_IMU__IMU_TEST_H_
