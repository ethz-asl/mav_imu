//
// Created by acey on 24.08.22.
// Modified by TimonMathis on 23.02.2024 for ROS2
//

#include <imu_node.h>
#include <log++.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

ImuNode::ImuNode(ImuInterface &imu, int frequency)
: Node("imu_node"), imu_interface_(imu), frequency_(frequency) {
  imu_data_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);
  imu_mag_pub_      = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 1);
  imu_temp_pub_     = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temp", 1);
  imu_baro_pub_     = this->create_publisher<sensor_msgs::msg::FluidPressure>("imu/pressure", 1);
}

int ImuNode::run() {
  LOG(I, "Node started");
  imu_interface_.printImuConfig();
  while (rclcpp::ok() && run_node) {
    rclcpp::Rate loop_rate(frequency_);

    time_now_         = this->now();
    imu_burst_result_ = imu_interface_.burst();

    processImuData();
    processMagneticFieldData();
    processTemperature();
    processFluidpressure();

    LOG_FIRST(I, 1, "Published first imu message");
    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
  }
  imu_interface_.close();
  return 0;
}

void ImuNode::processImuData() {
  if (imu_burst_result_.acceleration.has_value() && imu_burst_result_.gyro.has_value()) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = time_now_;
    msg.header.frame_id = "imu";

    msg.linear_acceleration.x = imu_burst_result_.acceleration.value().x;
    msg.linear_acceleration.y = imu_burst_result_.acceleration.value().y;
    msg.linear_acceleration.z = imu_burst_result_.acceleration.value().z;

    msg.angular_velocity.x = imu_burst_result_.gyro.value().x;
    msg.angular_velocity.y = imu_burst_result_.gyro.value().y;
    msg.angular_velocity.z = imu_burst_result_.gyro.value().z;
    imu_data_raw_pub_->publish(msg);
  }
}