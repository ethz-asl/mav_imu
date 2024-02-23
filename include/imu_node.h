//
// Created by acey on 24.08.22.
// Modified by TimonMathis on 23.02.2024 for ROS2
//

#ifndef MAV_IMU__IMU_TEST_H_
#define MAV_IMU__IMU_TEST_H_

#include "imu/adis16448.h"
#include "spi_driver.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <string>

class ImuNode : public rclcpp::Node {
 public:
  ImuNode(ImuInterface &imu_interface, int frequency);
  int run();

  inline static bool run_node = true;

 private:
  void processImuData();
  void processMagneticFieldData();
  void processTemperature();
  void processFluidpressure();

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr imu_mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imu_temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr imu_baro_pub_;
};
#endif  // MAV_IMU__IMU_TEST_H_