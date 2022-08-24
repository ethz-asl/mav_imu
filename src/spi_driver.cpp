//
// Created by acey on 22.08.22.
//

#include <fcntl.h>
#include "spi_driver.h"
#include <linux/spi/spidev.h>
#include <cstring>
#include <iostream>
#include <utility>
#include <sys/ioctl.h>

SpiDriver::SpiDriver(std::string path) : path_(std::move(path)) {}

bool SpiDriver::open() {

  fd_ = ::open(path_.data(), O_RDWR);
  if (fd_ < 0) {
    std::cout << "Error on open: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool SpiDriver::setMode(uint8_t mode) const {
  int res = ioctl(fd_, SPI_IOC_WR_MODE, &mode);

  if (res < 0) {
    std::cout << "ioctl SPI_IOC_WR_MODE failed: " << strerror(errno) << std::endl;
    return false;
  }
  return true;
}

std::vector<byte> SpiDriver::xfer(const std::vector<byte>& cmd) const {
  if (cmd.size() != 2) {
    return {};
  }

  struct spi_ioc_transfer xfer[1];
  unsigned char buf[32]{};

  xfer->speed_hz = 2000000;

  int len = 2;
  memset(xfer, 0, sizeof xfer);
  memset(buf, 0, sizeof buf);

// Send a read command
  buf[0] = cmd[0];
  buf[1] = cmd[1];

  xfer[0].tx_buf = (unsigned long) buf;
  xfer[0].len = len;

  unsigned char buf2[len];
  xfer[0].rx_buf = (unsigned long) buf2;
  xfer[0].len = len;

  int status = ioctl(fd_, SPI_IOC_MESSAGE(2), xfer);
  if (status < 0) {
    std::cout << "ioctl SPI_IOC_MESSAGE failed: " << strerror(errno) << std::endl;
    return {};
  }

  std::vector<unsigned char> res(len);

  for (int i = 0; i < len; i++) {
    res.push_back(buf2[i]);
  }

  std::cout << "response (" << status << "):" << std::endl;
  for (unsigned char *bp = buf2; len; len--) {
    std::cout << std::hex << (int) *bp++ << " ";
  }
  std::cout << std::endl;

  return res;
}