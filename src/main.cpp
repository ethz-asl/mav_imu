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
  ros::init(argc, argv, "mav_imu_node");

  ros::NodeHandle nh_private("~");
  std::string path = nh_private.param("spi_path", std::string("/dev/spidev0.1"));
  int frequency = nh_private.param("frequency", 200);

  LOG(I, "Spi path: " << path);
  LOG(I, "Loop frequency " << frequency);

  Adis16448 adis_16448{path};
  ImuNode node{adis_16448, frequency};
  if (!node.init()) {
    LOG(F, "Imu init failed.");
    return -1;
  }
  return node.run();
}