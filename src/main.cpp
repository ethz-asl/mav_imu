#include <csignal>
#include "imu_node.h"
#include <ros/ros.h>

void SignalHandler(int signum) {
  if (signum == SIGINT) {
    ROS_INFO("Received sigint. Shutting down.");
    ImuNode::run_node = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_node");
  signal(SIGINT, SignalHandler);
  ImuNode node{"/dev/spidev0.1"};
  if (!node.init()) {
    return -1;
  }
  return node.run();
}