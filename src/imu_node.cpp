//
// Created by acey on 24.08.22.
//
#include <log++/log++.h>
#include <ros/ros.h>
#include <imu_node.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>

ImuNode::ImuNode(const std::string &path, ImuInterface &imu) : imu_interface_(imu) {
  imu_data_raw_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 1);
  imu_mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag", 1);

}

bool ImuNode::init() {
  if (!imu_interface_.init()) {
    return false;
  }
  if (!imu_interface_.selftest()) {
    return true;
  }
  return true;
}

int ImuNode::run() {
  while (ros::ok() && run_node) {
    ros::Rate loop_rate(1);

    LOG(I, "Temperature: " << imu_interface_.getTemperature());
    LOG(I, "Baro: " << imu_interface_.getBarometer());
    LOG(I, "Gyro: " << imu_interface_.getGyro().toString());
    LOG(I, "Accel: " << imu_interface_.getAcceleration().toString());


    //TODO implement

    sensor_msgs::Imu imu_msg = processImuData();

    sensor_msgs::MagneticField mag_msg;

    mag_msg.header.stamp = ros::Time::now();

    vec3<int> mag = imu_interface_.getMagnetometer();
    mag_msg.magnetic_field.x = mag.x;
    mag_msg.magnetic_field.y = mag.y;
    mag_msg.magnetic_field.z = mag.z;


    imu_data_raw_pub_.publish(imu_msg);
    imu_mag_pub_.publish(mag_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

sensor_msgs::Imu ImuNode::processImuData() {
  ros::Time time = ros::Time::now();

  sensor_msgs::Imu msg;
  msg.header.stamp = time;

  auto a = imu_interface_.getAcceleration();
  msg.linear_acceleration.x = a.x;
  msg.linear_acceleration.y = a.y;
  msg.linear_acceleration.z = a.z;

  auto g = imu_interface_.getGyro();
  msg.angular_velocity.x = g.x;
  msg.angular_velocity.y = g.y;
  msg.angular_velocity.z = g.z;


  return msg;
}
