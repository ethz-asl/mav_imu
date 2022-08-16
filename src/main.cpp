#include <cstdio>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <iostream>

int main() {
  std::cout << "Test" << std::endl;
  char *name = "/dev/spidev0.1";
  struct spi_ioc_transfer xfer[2];
  unsigned char buf[32], *bp;
  int status;

  int fd = open(name, O_RDWR);
  if (fd < 0) {
    perror("open");
    return 1;
  }

  memset(xfer, 0, sizeof xfer);
  memset(buf, 0, sizeof buf);

/*
* Send a read command
*/
  buf[0] = 0x04;
  buf[1] = 0x00;

  xfer[0].tx_buf = (unsigned long) buf;
  xfer[0].len = 2;

  int len = 4;
  unsigned char buf2[len];
  xfer[1].rx_buf = (unsigned long) buf;
  xfer[1].len = 4;

  status = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
  if (status < 0) {
    perror("SPI_IOC_MESSAGE");
    return -1;
  }

  printf("response(%d): ", status);
  for (bp = buf2; len; len--)
    printf("%02x ", *bp++);
  printf("\n");

  return 0;
}