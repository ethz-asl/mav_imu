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
#include <unistd.h>
#include <log++/log++.h>

SpiDriver::SpiDriver(std::string path) : path_(std::move(path)) {}

bool SpiDriver::open() {

  fd_ = ::open(path_.data(), O_RDWR);
  if (fd_ < 0) {
    LOG(E, "Error on open: " << strerror(errno));
    return false;
  }
  return true;
}

bool SpiDriver::setMode(uint8_t mode) const {
  int res = ioctl(fd_, SPI_IOC_WR_MODE, &mode);

  if (res < 0) {
    LOG(E, "ioctl SPI_IOC_WR_MODE failed: " << strerror(errno));
    return false;
  }
  return true;
}

std::vector<byte> SpiDriver::burst(const std::vector<byte> &cmd, int res_len) {


  unsigned char buf[2];
  buf[0] = cmd[0];
  buf[1] = cmd[1];

  struct spi_ioc_transfer mesg[4];
  memset(mesg, 0, sizeof(mesg));


  for (int i = 0; i < 4; i++) {
    mesg[i].len = 2;
  }
  mesg[0].tx_buf = (__u64) buf;



  int status = ioctl(fd_, SPI_IOC_MESSAGE(4), &mesg);

  if (status < 0) {
    std::cout << "SPI_IOC_MESSAGE failed: " << strerror(errno) << std::endl;
    return {};
  }

  return {};
}


std::vector<byte> SpiDriver::xfer(const std::vector<byte> &cmd) const {
  if (cmd.size() > sizeof(__u64)) {
    LOG(E, "cmd buffer to big " << cmd.size() << " > " << sizeof(__u64));
    return {};
  }

  struct spi_ioc_transfer xfer[2];
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
  memset(buf2, 0, sizeof buf2);
  xfer[1].rx_buf = (unsigned long) buf2;
  xfer[1].len = len;

  int status = ioctl(fd_, SPI_IOC_MESSAGE(2), xfer);
  if (status < 0) {
    LOG(E, "ioctl SPI_IOC_MESSAGE failed: " << strerror(errno));
    return {};
  }

  std::vector<unsigned char> res{};

  for (int i = 0; i < len; i++) {
    res.push_back(buf2[i]);
  }


  std::stringstream ss;
  ss << "response (" << status / 2 << "): ";
  for (unsigned char *bp = buf2; len; len--) {
    ss << std::hex << (int) *bp++ << " ";
  }
  LOG(I, ss.str());
  return res;
}

bool SpiDriver::close() const {
  if (::close(fd_) != 0) {
    LOG(I, "Error closing fd: " << strerror(errno));
    return false;
  }
  return true;
}

SpiDriver::~SpiDriver() {
  ::close(fd_);
}

