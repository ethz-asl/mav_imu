//
// Created by acey on 22.08.22.
//

#ifndef MAV_IMU_SRC_SPI_DRIVER_H_
#define MAV_IMU_SRC_SPI_DRIVER_H_

#include <string>
#include <vector>
#include "spi_commands.h"

class SpiDriver {
 public:
  explicit SpiDriver(std::string path);
  bool open();
  int xfer() const;


 private:
  int fd_{};
  std::string path_;
};

#endif //MAV_IMU_SRC_SPI_DRIVER_H_
