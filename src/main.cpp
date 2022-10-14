#include "imu/ImuFactory.h"
#include "imu/adis16448.h"
#include "imu_node.h"
#include <csignal>
#include <log++.h>
#include <ros/ros.h>

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
  std::string spi_path =
      nh_private.param("spi_path", std::string("/dev/spidev0.1"));
  int frequency = nh_private.param("frequency", 200);
  std::string imu_name = nh_private.param("imu", std::string("adis16448"));

  LOG(I, "Spi path: " << spi_path);
  LOG(I, "Loop frequency " << frequency);

  ImuInterface *imu_interface = ImuFactory::createImuByName(imu_name, spi_path);
  ImuNode node{*imu_interface, frequency};
  if (!node.init()) {
    LOG(F, "Imu init failed.");
    return -1;
  }
  node.run();

  delete imu_interface; // Call destructor to close spi device
  return 0;
}