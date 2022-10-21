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
   * Spi half-duplex transaction
   *
   * @param cmd vector with multiple bytes
   * @param response_len response length
   * @param speed_hz SPI transfer clock speed
   * @return vector with response or empty vector on failure
   */
  [[nodiscard]] std::vector<byte> xfer(const std::vector<byte> &cmd,
                                       const int response_len,
                                       const uint32_t speed_hz) const;

  /**
   * Spi duplex transaction
   *
   * @param cmds multiple commands with multiple bytes
   * @param speed_hz SPI clock speed
   * @return output of each command in command order
   */
  [[nodiscard]] std::vector<std::vector<byte>>
  xfer2(const std::vector<std::vector<byte>> &cmds,
        const uint32_t speed_hz) const;

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

#endif // MAV_IMU_SRC_SPI_DRIVER_H_
