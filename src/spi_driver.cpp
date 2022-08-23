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

SpiDriver::SpiDriver(std::string path) : path_(std::move(path)) {

}
bool SpiDriver::open() {

  fd_ = ::open(path_.data(), O_RDWR);
  if (fd_ < 0) {
    std::cout << "Error on open: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

int SpiDriver::xfer() const {
  struct spi_ioc_transfer xfer[1];
  unsigned char buf[32]{};
  unsigned char *bp;


  memset(xfer, 0, sizeof xfer);
  memset(buf, 0, sizeof buf);

// Send a read command
  buf[0] = 0x56;
  buf[1] = 0x00;

  xfer[0].tx_buf = (unsigned long) buf;
  xfer[0].len = 2;

  int len = 2;
  unsigned char buf2[len];
  xfer[0].rx_buf = (unsigned long) buf2;
  xfer[0].len = len;

  uint8_t a = SPI_MODE_3;
  int res = ioctl(fd_, SPI_IOC_WR_MODE, &a);

  if (res < 0) {
    perror("SPI_MODE_3");
    return -1;
  }

  int status = ioctl(fd_, SPI_IOC_MESSAGE(2), xfer);
  if (status < 0) {
    perror("SPI_IOC_MESSAGE");
    return -1;
  }

  printf("response(%d): ", status);
  for (bp = buf2; len; len--) {
    printf("%02x ", *bp++);
  }
  std::cout << std::endl;

  return 0;
}
