//
// Created by acey on 24.08.22.
//
#include <log++/log++.h>
#include <ros/ros.h>
#include <imu_node.h>

ImuNode::ImuNode(ImuInterface &imu, int frequency) : imu_interface_(imu), frequency_(frequency) {
  imu_data_raw_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 1);
  imu_mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag", 1);
  imu_temp_pub_ = nh_.advertise<sensor_msgs::Temperature>("/imu/temp", 1);
  imu_baro_pub_ = nh_.advertise<sensor_msgs::FluidPressure>("/imu/pressure", 1);
}

bool ImuNode::init() {

  if (!imu_interface_.init()) {
    return false;
  }
  if (!imu_interface_.selftest()) {
    return true;
  }
  LOG(I, "Imu Node initalized.");
  return true;
}

int ImuNode::run() {
  LOG(I, "Node started");
  while (ros::ok() && run_node) {
    ros::Rate loop_rate(frequency_);

    //TODO implement
    ros::Time time = ros::Time::now();
    sensor_msgs::Imu imu_msg = processImuData(time);
    sensor_msgs::MagneticField mag_msg = processMagneticFieldData(time);
    sensor_msgs::Temperature  temp_msg = processTemperature(time);
    sensor_msgs::FluidPressure pressure_msg = processFluidpressure(time);

    imu_data_raw_pub_.publish(imu_msg);
    imu_mag_pub_.publish(mag_msg);
    imu_temp_pub_.publish(temp_msg);
    imu_baro_pub_.publish(pressure_msg);

    LOG_FIRST(I, 1, "Published first imu message");
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

sensor_msgs::Imu ImuNode::processImuData(ros::Time time) {
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

sensor_msgs::MagneticField ImuNode::processMagneticFieldData(ros::Time time) {
  sensor_msgs::MagneticField mag_msg;

  mag_msg.header.stamp = time;

  vec3<int> mag = imu_interface_.getMagnetometer();
  mag_msg.magnetic_field.x = mag.x;
  mag_msg.magnetic_field.y = mag.y;
  mag_msg.magnetic_field.z = mag.z;
  return mag_msg;
}

sensor_msgs::Temperature ImuNode::processTemperature(ros::Time time) {
  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = time;
  temp_msg.temperature = imu_interface_.getTemperature();
  temp_msg.variance = 0;

  return temp_msg;
}
sensor_msgs::FluidPressure ImuNode::processFluidpressure(ros::Time time) {
  sensor_msgs::FluidPressure pressure_msg;
  pressure_msg.header.stamp = time;
  //Ros takes fluid pressure in Pa. Convert hPa to Pa.
  pressure_msg.fluid_pressure = imu_interface_.getBarometer() * 100;
  pressure_msg.variance = 0;
  return pressure_msg;
}
