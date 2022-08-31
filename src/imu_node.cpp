//
// Created by acey on 24.08.22.
//

#include <ros/ros.h>
#include "imu_node.h"
#include "sensor_msgs/Imu.h"
#include "log++/log++.h"

ImuNode::ImuNode(const std::string &path, ImuInterface& imu) : imu_interface_(imu)  {}

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


    //TODO implement
    sensor_msgs::Imu msg;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
