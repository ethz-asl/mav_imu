//
// Created by acey on 25.08.22.
//

#ifndef MAV_IMU_SRC_IMU_ADIS16448_H_
#define MAV_IMU_SRC_IMU_ADIS16448_H_
#include <string>

#include "imu_interface.h"
#include "spi_driver.h"

class Adis16448 : public ImuInterface {
 public:
  /**
   * Adis16448 Constructor
   * @param path to spidev e.g. /dev/spidev0.1
   */
  explicit Adis16448(const std::string &path);

  /**
   * Adis16448 Destructor
   */
  ~Adis16448();

  /**
   * Enable crc checksum check on burst read
   * @param b
   * @return true if successful, otherwise false
   */
  bool setBurstCRCEnabled(bool b);

  bool selftest() override;
  bool init() override;
  std::optional<vec3<double>> getGyro() override;
  std::optional<vec3<double>> getAcceleration() override;
  std::optional<vec3<double>> getMagnetometer() override;
  std::optional<double> getBarometer() override;

  /**
   * Note that this temperature represents
   * an internal temperature reading, which does not precisely
   * represent external conditions. The intended use of TEMP_OUT
   * is to monitor relative changes in temperature.
   */
  std::optional<double> getTemperature() override;
  int getRaw(std::vector<byte> cmd) override;

  /**
   * Custom burst mode
   * @return struct with all values.
   */
  ImuBurstResult burst() override;

  bool calibrateGyro() override;
  /**
   * Free file descriptor
   * @return true if successful, otherwise false and errno is set.
   */
  bool close() final;

  static int signedWordToInt(const std::vector<byte> &word);
  static int unsignedWordToInt(const std::vector<byte> &word);
  static bool validateCrc(const std::vector<byte> &burstData);

 private:
  static unsigned short int runCRC(const uint16_t burstData[]);
  static inline const constexpr int DEFAULT_BURST_LEN = 24;

  /**
   * Helper function to read a registry entry.
   */
  std::vector<byte> readReg(uint8_t addr);

  /**
   * Helper function to overwrite a registry entry.
   */
  void writeReg(uint8_t addr, const std::vector<byte> &data,
                const std::string &name);

  /**
   * Run a test read sequence for SPI communication.
   */
  bool testSPI();

  //! Convert spi output to measurement unit required by the ImuInterface

  /**
   * @param gyro
   * @return rad/s
   */
  static vec3<double> convertGyro(vec3<double> gyro);

  /**
   * @param accel
   * @return m/s^2
   */
  static vec3<double> convertAcceleration(vec3<double> accel);

  /**
   * @param magnetometer
   * @return tesla [T]
   */
  static vec3<double> convertMagnetometer(vec3<double> magnetometer);

  /**
   * @param word
   * @return
   */
  static double convertBarometer(const std::vector<byte> &word);

  /**
   * @param word
   * @return
   */
  static double convertTemperature(const std::vector<byte> &word);

  SpiDriver spi_driver_;
  int burst_len_{DEFAULT_BURST_LEN};
  int crc_error_count_{0};

  inline static const constexpr uint32_t spi_transfer_speed_hz_ = 2000000;
  inline static const constexpr uint32_t spi_burst_speed_hz_    = 1000000;
  inline static const constexpr uint32_t spi_response_size_     = 2;
  inline static const constexpr uint32_t ms_                    = 100e3;
};

#endif // MAV_IMU_SRC_IMU_ADIS16448_H_
