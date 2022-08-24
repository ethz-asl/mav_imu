//
// Created by acey on 24.08.22.
//

#include <linux/spi/spidev.h>
#include <ros/ros.h>
#include "imu_node.h"
#include "sensor_msgs/Imu.h"
ImuNode::ImuNode(const std::string &path) : spi_driver_(path) {

}
bool ImuNode::init() {
  if (!spi_driver_.open()) {
    return false;
  }

  if (!spi_driver_.setMode(SPI_MODE_3)) {
    return false;
  }
  return true;
}

int ImuNode::run() {
  while (ros::ok() && run_node) {
    ros::Rate loop_rate(1);
    if (!spi_driver_.xfer()) {
      return -1;
    }

    //TODO implement
    sensor_msgs::Imu msg;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
