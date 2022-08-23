#include "spi_driver.h"


int main() {
  SpiDriver spi_driver("/dev/spidev0.1");

  if (spi_driver.open()) {
    spi_driver.xfer();
  } else {
    return -1;
  }

  return 0;
}