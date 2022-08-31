#include <csignal>
#include "imu_node.h"
#include <ros/ros.h>
#include "imu/adis16448.h"
#include <log++/log++.h>

void SignalHandler(int signum) {
  if (signum == SIGINT) {
    LOG(I, "Received sigint. Shutting down.");
    ImuNode::run_node = false;
  }
}


int main(int argc, char **argv) {
  LOG_INIT(argv[0]);
  signal(SIGINT, SignalHandler);
  std::string path = "/dev/spidev0.1";

  Adis16448 test{path};
  LOG(I, test.unsignedWordToInt({0x01, 0x00}));

  ros::init(argc, argv, "test_node");
  ImuNode node{path, test};
  if (!node.init()) {
    return -1;
  }
  return node.run();
}