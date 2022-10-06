//
// Created by acey on 24.08.22.
//
#include <log++.h>
#include <ros/ros.h>
#include <imu_node.h>

ImuNode::ImuNode(ImuInterface &imu, int frequency) : imu_interface_(imu), frequency_(frequency) {
  imu_data_raw_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 1);
  imu_mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag", 1);
  imu_temp_pub_ = nh_.advertise<sensor_msgs::Temperature>("/imu/temp", 1);
  imu_baro_pub_ = nh_.advertise<sensor_msgs::FluidPressure>("/imu/pressure", 1);
}

bool ImuNode::init() {
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


    time_now_ = ros::Time::now();
    imu_burst_result_ = imu_interface_.burst();

    sensor_msgs::Imu imu_msg = processImuData();
    sensor_msgs::MagneticField mag_msg = processMagneticFieldData();
    sensor_msgs::Temperature  temp_msg = processTemperature();
    sensor_msgs::FluidPressure pressure_msg = processFluidpressure();

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

sensor_msgs::Imu ImuNode::processImuData() {
  sensor_msgs::Imu msg;
  msg.header.stamp = time_now_;
  msg.header.frame_id = "map";

  msg.linear_acceleration.x = imu_burst_result_.acceleration.x;
  msg.linear_acceleration.y = imu_burst_result_.acceleration.y;
  msg.linear_acceleration.z = imu_burst_result_.acceleration.z;

  msg.angular_velocity.x = imu_burst_result_.gyro.x;
  msg.angular_velocity.y = imu_burst_result_.gyro.y;
  msg.angular_velocity.z = imu_burst_result_.gyro.z;
  return msg;
}

sensor_msgs::MagneticField ImuNode::processMagneticFieldData() {
  sensor_msgs::MagneticField mag_msg;

  mag_msg.header.stamp = time_now_;

  mag_msg.magnetic_field.x = imu_burst_result_.magnetometer.x;
  mag_msg.magnetic_field.y = imu_burst_result_.magnetometer.y;
  mag_msg.magnetic_field.z = imu_burst_result_.magnetometer.z;
  return mag_msg;
}

sensor_msgs::Temperature ImuNode::processTemperature() {
  sensor_msgs::Temperature temp_msg;
  temp_msg.header.stamp = time_now_;
  temp_msg.temperature = imu_burst_result_.temp;
  temp_msg.variance = 0;

  return temp_msg;
}

sensor_msgs::FluidPressure ImuNode::processFluidpressure() {
  sensor_msgs::FluidPressure pressure_msg;
  pressure_msg.header.stamp = time_now_;
  //Ros takes fluid pressure in Pa. Convert hPa to Pa.
  pressure_msg.fluid_pressure = imu_burst_result_.baro * 100;
  pressure_msg.variance = 0;
  return pressure_msg;
}
