//
// Created by acey on 22.08.22.
//

#include <fcntl.h>
#include <spi_driver.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <utility>
#include <sys/ioctl.h>
#include <unistd.h>
#include <log++.h>

SpiDriver::SpiDriver(std::string path) : path_(std::move(path)) {}

bool SpiDriver::open() {

  fd_ = ::open(path_.data(), O_RDWR);
  if (fd_ < 0) {
    LOG(E, "Error on open: " << strerror(errno));
    return false;
  }
  is_open_ = true;
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

std::vector<std::vector<byte>> SpiDriver::xfer2(const std::vector<std::vector<byte>> &cmds, const uint32_t speed_hz) const {

  std::vector<std::vector<byte>> res{};
  for (int i = 0; i < cmds.size() + 1; i++) {

    std::vector<byte> cmd{};
    if (i < cmds.size()) {
      const std::vector<byte>& a = cmds.at(i);
      cmd = a;
    } else {
      cmd = {0x00, 0x00};
    }

    if (cmd.size() > sizeof(__u64)) {
      LOG(E, "cmd buffer to big " << cmd.size() << " > " << sizeof(__u64));
      return {};
    }

    struct spi_ioc_transfer xfer[1];
    unsigned char buf[32]{};

    xfer->speed_hz = speed_hz;

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
    xfer[0].rx_buf = (unsigned long) buf2;
    xfer[0].len = len;

    int status = ioctl(fd_, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
      LOG(E, "ioctl SPI_IOC_MESSAGE failed: " << strerror(errno));
      return {};
    }

    if (i == 0) {
      continue;
    }

    std::vector<unsigned char> ret{};

    for (int j = 0; j < len; j++) {
      ret.push_back(buf2[j]);
    }
    res.push_back(ret);
  }
  return res;
}

std::vector<byte> SpiDriver::xfer(const std::vector<byte> &cmd, const uint32_t speed_hz, int response_len) const {
  if (cmd.size() > 32) {
    LOG(E, "cmd buffer to big " << cmd.size() << " > " << 32);
    return {};
  }

  struct spi_ioc_transfer xfer[2];
  unsigned char buf[32]{};

  xfer->speed_hz = speed_hz;

  int len = (int) cmd.size();
  memset(xfer, 0, sizeof xfer);
  memset(buf, 0, sizeof buf);

  // Send a read command
  for (int i = 0; i < cmd.size(); i++) {
    buf[i] = cmd[i];
  }

  xfer[0].tx_buf = (unsigned long) buf;
  xfer[0].len = cmd.size();

  unsigned char buf2[len];
  memset(buf2, 0, sizeof buf2);
  xfer[1].rx_buf = (unsigned long) buf2;
  xfer[1].len = response_len;

  int status = ioctl(fd_, SPI_IOC_MESSAGE(2), xfer);
  if (status < 0) {
    LOG(E, "ioctl SPI_IOC_MESSAGE failed: " << strerror(errno));
    return {};
  }

  std::vector<unsigned char> res{};

  for (int i = 0; i < len; i++) {
    res.push_back(buf2[i]);
  }
  return res;
}

bool SpiDriver::close() {
  if (::close(fd_) != 0) {
    LOG(I, "Error closing fd: " << strerror(errno));
    return false;
  }
  is_open_ = false;
  return true;
}

SpiDriver::~SpiDriver() {
  ::close(fd_);
}

bool SpiDriver::isOpen() const {
  return is_open_;
}

int SpiDriver::getFd() const {
  return fd_;
}

const std::string &SpiDriver::getPath() const {
  return path_;
}

