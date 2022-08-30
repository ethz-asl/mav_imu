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


  Adis16448 adis_16448(path);
  if (!adis_16448.init()) {
    LOG(E, "init failed.");
    return -1;
  }
  LOG(I, "Adis16448 initialized");

  adis_16448.selftest();
  adis_16448.burstread();
  adis_16448.burstread();
  adis_16448.close();

  ros::init(argc, argv, "test_node");
  ImuNode node{path};
  if (!node.init()) {
    return -1;
  }
  return node.run();
}