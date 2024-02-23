#include "imu/ImuFactory.h"
#include "imu/adis16448.h"
#include "imu_node.h"
#include <csignal>
#include <log++.h>
#include <rclcpp/rclcpp.hpp>

void SignalHandler(int signum) {
  if (signum == SIGINT) {
    LOG(I, "Received sigint. Shutting down.");
    ImuNode::run_node = false;
  }
}

int main(int argc, char **argv) {
  LOG_INIT(argv[0]);
  rclcpp::init(argc, argv);

  auto ros_node = std::make_shared<rclcpp::Node>("mav_imu_node");
  std::string spi_path = ros_node->declare_parameter("spi_path", "/dev/spidev0.0");
  int frequency = ros_node->declare_parameter("frequency", 200);
  std::string imu_name = ros_node->declare_parameter("imu", "bmi088");

  LOG(I, "Spi path: " << spi_path);
  LOG(I, "Loop frequency " << frequency);

  ImuInterfacePtr imu_interface = ImuFactory::createImuByName(imu_name, spi_path);
  if (imu_interface == nullptr) {
    LOG(F, "Imu interface failed to initialize");
    return -1;
  }

  ImuNode node{*imu_interface, frequency};

  rclcpp::spin(ros_node);
  rclcpp::shutdown();
  return 0;
}