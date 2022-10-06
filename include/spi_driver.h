//
// Created by acey on 22.08.22.
//

#ifndef MAV_IMU_SRC_SPI_DRIVER_H_
#define MAV_IMU_SRC_SPI_DRIVER_H_

#include <string>
#include <vector>

typedef unsigned char byte;

class SpiDriver {
 public:
  explicit SpiDriver(std::string path);
  bool open();
  [[nodiscard]] bool setMode(uint8_t mode) const;

  /**
   * Spi transaction
   *
   * @param cmd vector with multiple bytes, defaults to 2 for compatibility reasons
   * @return vector with response or empty vector on failure
   */
  [[nodiscard]] std::vector<byte> xfer(const std::vector<byte>& cmd, int response_len = 2) const;
  [[nodiscard]] std::vector<std::vector<byte>> burst(const std::vector<std::vector<byte>>& cmd) const;

  bool close();

  ~SpiDriver();

  //! Getters
  [[nodiscard]] bool isOpen() const;
  [[nodiscard]] int getFd() const;
  [[nodiscard]] const std::string &getPath() const;
 private:
  bool is_open_{false};
  int fd_{};
  std::string path_;
};

#endif //MAV_IMU_SRC_SPI_DRIVER_H_
