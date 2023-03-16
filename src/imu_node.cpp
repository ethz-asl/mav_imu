//
// Created by acey on 24.08.22.
//
#include <imu_node.h>
#include <log++.h>
#include <ros/ros.h>

ImuNode::ImuNode(ImuInterface &imu, int frequency)
    : imu_interface_(imu), frequency_(frequency) {
  ros::NodeHandle nh;
  imu_data_raw_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
  imu_mag_pub_      = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
  imu_temp_pub_     = nh.advertise<sensor_msgs::Temperature>("imu/temp", 1);
  imu_baro_pub_     = nh.advertise<sensor_msgs::FluidPressure>("imu/pressure", 1);
}

int ImuNode::run() {
  LOG(I, "Node started");
  imu_interface_.printImuConfig();
  while (ros::ok() && run_node) {
    ros::Rate loop_rate(frequency_);

    time_now_         = ros::Time::now();
    imu_burst_result_ = imu_interface_.burst();

    processImuData();
    processMagneticFieldData();
    processTemperature();
    processFluidpressure();

    LOG_FIRST(I, 1, "Published first imu message");
    ros::spinOnce();
    loop_rate.sleep();
  }
  imu_interface_.close();
  return 0;
}

void ImuNode::processImuData() {
  if (imu_burst_result_.acceleration.has_value() && imu_burst_result_.gyro.has_value()) {
    sensor_msgs::Imu msg;
    msg.header.stamp    = time_now_;
    msg.header.frame_id = "imu";

    msg.linear_acceleration.x = imu_burst_result_.acceleration.value().x;
    msg.linear_acceleration.y = imu_burst_result_.acceleration.value().y;
    msg.linear_acceleration.z = imu_burst_result_.acceleration.value().z;

    msg.angular_velocity.x = imu_burst_result_.gyro.value().x;
    msg.angular_velocity.y = imu_burst_result_.gyro.value().y;
    msg.angular_velocity.z = imu_burst_result_.gyro.value().z;
    imu_data_raw_pub_.publish(msg);
  }
}

void ImuNode::processMagneticFieldData() {
  if (imu_burst_result_.magnetometer.has_value()) {

    sensor_msgs::MagneticField mag_msg;

    mag_msg.header.stamp = time_now_;

    mag_msg.magnetic_field.x = imu_burst_result_.magnetometer.value().x;
    mag_msg.magnetic_field.y = imu_burst_result_.magnetometer.value().y;
    mag_msg.magnetic_field.z = imu_burst_result_.magnetometer.value().z;

    imu_mag_pub_.publish(mag_msg);
  }
}

void ImuNode::processTemperature() {
  if (imu_burst_result_.temp.has_value()) {

    sensor_msgs::Temperature temp_msg;
    temp_msg.header.stamp = time_now_;
    temp_msg.temperature  = imu_burst_result_.temp.value();
    temp_msg.variance     = 0;
    imu_temp_pub_.publish(temp_msg);
  }
}

void ImuNode::processFluidpressure() {

  if (imu_burst_result_.baro.has_value()) {
    sensor_msgs::FluidPressure pressure_msg;
    pressure_msg.header.stamp = time_now_;
    // Ros takes fluid pressure in Pa. Convert hPa to Pa.
    pressure_msg.fluid_pressure = imu_burst_result_.baro.value() * 100;
    pressure_msg.variance       = 0;

    imu_baro_pub_.publish(pressure_msg);
  }
}
