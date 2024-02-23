//
// Created by acey on 25.08.22.
//

#include "imu/adis16448.h"
#include "imu/adis16448_cmds.h"
#include <cstring>
#include <iostream>
#include <linux/spi/spidev.h>
#include <log++.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <climits>

#define _USE_MATH_DEFINES
#include <cmath>


Adis16448::Adis16448(const std::string &path) : spi_driver_(path) {}

Adis16448::~Adis16448() {
  // TODO(rikba): This does not work...
  std::vector<byte> gpio_ctrl = readReg(GPIO_CTRL);
  gpio_ctrl[1] &= ~(1 << 1); // Clear DIO2 output to disable LED.
  writeReg(GPIO_CTRL, gpio_ctrl, "GPIO_CTRL");
}

bool Adis16448::init() {

  if (!spi_driver_.open()) {
    LOG(E, "open failed: " << strerror(errno));
    return false;
  }

  if (!spi_driver_.setMode(SPI_MODE_3)) {
    LOG(E, "Setmode failed");
    return false;
  }

  if (!testSPI()) {
    LOG(E, "SPI test read failed.");
    return false;
  }

  // Selftest
  if (!selftest()) { return false; }

  // Software reset.
  softwareReset();

  // Calibration factory reset.
  LOG(I, "Adis16448 factory calibration.");
  writeReg(GLOB_CMD, {0x0, 1 << 1}, "GLOB_CMD");
  usleep(ms_);

  // TODO(rikba): Gyro auto-calibration.

  // General configuration.
  LOG(I, "Adis16448 configuration.");
  std::vector<byte> msc_ctrl = {0x00, 0x06};
  writeReg(MSC_CTRL, msc_ctrl, "MSC_CTRL");

  std::vector<byte> smpl_prd = {0x00, 0x01};
  // Factor 2 decimation to reduce update cycles (bad CRC).
  // TODO(rikba): Remove when DR handling is done.
  smpl_prd[0] |= (0b00001 << 0);
  writeReg(SMPL_PRD, smpl_prd, "SMPL_PRD");

  std::vector<byte> sens_avg = {0x04, 0x04};
  //sens_avg[1] &= ~(0b111 << 0); // Clear digital filter.
  writeReg(SENS_AVG, sens_avg, "SENS_AVG");

  writeReg(ALM_CTRL, {0x00, 0x00}, "ALM_CTRL");

  std::vector<byte> gpio_ctrl = {0x00, 0x00};
  gpio_ctrl[0] &= ~(1 << 1); // Clear DIO2 to light LED.
  gpio_ctrl[1] |= (1 << 1);  // Set DIO2 output.
  writeReg(GPIO_CTRL, gpio_ctrl, "GPIO_CTRL");

  return true;
}

std::vector<byte> Adis16448::readReg(const uint8_t addr) const {
  return spi_driver_.xfer(CMD(addr), spi_response_size_, spi_transfer_speed_hz_);
}

void Adis16448::writeReg(uint8_t addr, const std::vector<byte> &data,
                         const std::string &name) const {
  LOG(I, std::hex << "Adis16448 " << name.c_str() << ": 0x" << +data[0] << ", 0x" << +data[1]);
  // Set MSB
  addr = (addr & 0x7F) | 0x80;
  // Send low word.
  auto ret = spi_driver_.xfer({addr, data[1]}, 0, spi_transfer_speed_hz_);
  // Increment address.
  addr = (addr | 0x1);
  // Send high word.
  ret = spi_driver_.xfer({addr, data[0]}, 0, spi_transfer_speed_hz_);
}

bool Adis16448::selftest() {
  // Start self test.
  LOG(I, "Adis16448 self-test.");
  auto msc_ctrl = readReg(MSC_CTRL);
  msc_ctrl[0]   = (1 << 2) | msc_ctrl[0]; // Set bit 10 (3rd high bit).
  writeReg(MSC_CTRL, msc_ctrl, "MSC_CTRL");

  while (msc_ctrl[0] & (1 << 2)) {
    LOG(D, "Testing.");
    usleep(ms_); // Self test requires 45ms. Wait 100ms.
    msc_ctrl = readReg(MSC_CTRL);
  }

  std::vector<byte> res = readReg(DIAG_STAT);

  if (res.empty()) { return false; }

  if (res[1] & (1 << 5)) {
    LOG(E, "ADIS16448 self-test failed.");
    LOG(E, res[1] & (1 << 0), "Magnetometer functional test failure.");
    LOG(E, res[1] & (1 << 1), "Barometer functional test failure.");
    LOG(E, res[0] & (1 << 2), "X-axis gyroscope self-test failure.");
    LOG(E, res[0] & (1 << 3), "Y-axis gyroscope self-test failure.");
    LOG(E, res[0] & (1 << 4), "Z-axis gyroscope self-test failure.");
    LOG(E, res[0] & (1 << 5), "X-axis accelerometer self-test failure.");
    LOG(E, res[0] & (1 << 6), "Y-axis accelerometer self-test failure.");
    LOG(E, res[0] & (1 << 7), "Z-axis accelerometer self-test failure.");

    return false;
  }

  LOG(I, "Adis16448 self-check passed");
  return true;
}

bool Adis16448::testSPI() {
  auto res = readReg(PROD_ID);

  if (res.empty()) { return false; }

  LOG(I, std::hex << "Adis16448 PROD_ID: 0x" << +res[0] << +res[1]);

  return res[0] == 0x40 && res[1] == 0x40;
}

void Adis16448::softwareReset() {
  LOG(I, "Adis16448 software reset.");
  writeReg(GLOB_CMD, {0x0, 1 << 7}, "GLOB_CMD");
  usleep(ms_);
}

bool Adis16448::close() {
  softwareReset();
  return spi_driver_.close();
}

std::optional<vec3<double>> Adis16448::getGyro() {
  // twos complement format, 25 LSB/°/sec, 0°/sec = 0x0000
  vec3<double> gyro{};

  gyro.x = signedWordToInt(readReg(XGYRO_OUT));
  gyro.y = signedWordToInt(readReg(YGYRO_OUT));
  gyro.z = signedWordToInt(readReg(ZGYRO_OUT));

  return convertGyro(gyro);
}

vec3<double> Adis16448::convertGyro(vec3<double> gyro) {
  gyro /= 25.;                 // convert to degrees
  return gyro * (M_PI / 180.); // Convert to rad/s and return
}

std::optional<vec3<double>> Adis16448::getAcceleration() {
  // twos complement format, 1200 LSB/g, 0 g = 0x0000
  vec3<double> acceleration{};

  acceleration.x = signedWordToInt(readReg(XACCL_OUT));
  acceleration.y = signedWordToInt(readReg(YACCL_OUT));
  acceleration.z = signedWordToInt(readReg(ZACCL_OUT));

  return convertAcceleration(acceleration);
}

vec3<double> Adis16448::convertAcceleration(vec3<double> accel) {
  accel /= 1200.; // Convert to g
  return accel * g_;
}

std::optional<vec3<double>> Adis16448::getMagnetometer() {
  // twos complement, 7 LSB/mgauss, 0x0000 = 0 mgauss
  vec3<double> magnetometer{};

  magnetometer.x = signedWordToInt(readReg(XMAGN_OUT));
  magnetometer.y = signedWordToInt(readReg(YMAGN_OUT));
  magnetometer.z = signedWordToInt(readReg(ZMAGN_OUT));

  return convertMagnetometer(magnetometer);
}

vec3<double> Adis16448::convertMagnetometer(vec3<double> magnetometer) {
  magnetometer /= 7.;        // Convert to mG;
  magnetometer /= 10000000.; // Convert to tesla
  return magnetometer;
}

std::optional<double> Adis16448::getBarometer() {
  // 20 μbar per LSB, 0x0000 = 0 mbar
  int res = unsignedWordToInt(readReg(BARO_OUT));
  return res * 0.02;
}

double Adis16448::convertBarometer(const std::vector<byte> &word) {
  return unsignedWordToInt(word) * 0.02;
}

std::optional<double> Adis16448::getTemperature() {
  // Twos complement, 0.07386°C/LSB, 31°C = 0x0000, 12bit
  int a = signedWordToInt(readReg(TEMP_OUT));
  return 31 + (a * 0.07386);
}

double Adis16448::convertTemperature(const std::vector<byte> &word) {
  return 31 + (signedWordToInt(word) * 0.07386);
}

int Adis16448::getRaw(std::vector<byte> cmd) {
  std::vector<byte> res = spi_driver_.xfer(cmd, spi_response_size_, spi_transfer_speed_hz_);
  return unsignedWordToInt(res);
}

int Adis16448::unsignedWordToInt(const std::vector<byte> &word) {
  return ((word[0] << CHAR_BIT) + word[1]);
}

int Adis16448::signedWordToInt(const std::vector<byte> &word) {
  return (((int) *(signed char *) (word.data())) * 1 << CHAR_BIT) | word[1];
}

bool Adis16448::setBurstCRCEnabled(bool b) {
  if (b) {
    auto msc_ctrl = readReg(MSC_CTRL);
    msc_ctrl[1]   = (1 << 4) | msc_ctrl[1]; // Set lower bit 4.
    writeReg(MSC_CTRL, msc_ctrl, "MSC_CTRL");
    usleep(ms_); // wait 1ms
    auto res = readReg(MSC_CTRL);

    if (res[1] & (1 << 4)) {
      // increase rx buffer length by 2 bytes for 16bit crc value
      burst_len_ = DEFAULT_BURST_LEN + 2;
      LOG(I, "Enabled CRC on burst");
      return true;
    }

    LOG(E, "Error on burst mode enable");
    return false;
  } else {
    auto msc_ctrl = readReg(MSC_CTRL);
    msc_ctrl[1]   = (~(1 << 4)) & msc_ctrl[1]; // Clear lower bit 4.
    writeReg(MSC_CTRL, msc_ctrl, "MSC_CTRL");
    usleep(ms_); // wait 1ms
    auto res = readReg(MSC_CTRL);

    if (!(res[1] & (1 << 4))) {
      burst_len_ = DEFAULT_BURST_LEN;

      LOG(I, "Disabled CRC on burst");
      return true;
    }

    LOG(E, "Error on burst mode disable: " << (int) res[0] << ", " << (int) res[1]);
    return false;
  }
}

ImuBurstResult Adis16448::burst() {
  auto res = spi_driver_.xfer(CMD(GLOB_CMD), burst_len_, spi_burst_speed_hz_);

  if (burst_len_ == DEFAULT_BURST_LEN + 2 && !validateCrc(res)) {
    crc_error_count_++;

    // Since the adis is not synced with the host pc,
    // it is normal to have occasional checksum errors
    if (crc_error_count_ >= 5) {
      LOG_TIMED(E, 1,
                "DANGER: Last " << crc_error_count_
                                << " crc checks failed. Possible connection loss.");
    } else {
      LOG_EVERY(W, 1000, "Reported occasional checksum errors.");
    }
    return {};
  }
  crc_error_count_ = 0;

  vec3<double> gyro_raw{};
  gyro_raw.x = signedWordToInt({res[2], res[3]});
  gyro_raw.y = signedWordToInt({res[4], res[5]});
  gyro_raw.z = signedWordToInt({res[6], res[7]});

  vec3<double> raw_accel{};
  raw_accel.x = (double) signedWordToInt({res[8], res[9]});
  raw_accel.y = (double) signedWordToInt({res[10], res[11]});
  raw_accel.z = (double) signedWordToInt({res[12], res[13]});

  vec3<double> raw_magn{};
  raw_magn.x = signedWordToInt({res[14], res[15]});
  raw_magn.y = signedWordToInt({res[16], res[17]});
  raw_magn.z = signedWordToInt({res[18], res[19]});

  struct ImuBurstResult ret {};
  ret.gyro         = convertGyro(gyro_raw);
  ret.acceleration = convertAcceleration(raw_accel);
  ret.magnetometer = convertMagnetometer(raw_magn);

  ret.baro = convertBarometer({res[20], res[21]});
  ret.temp = convertTemperature({res[22], res[23]});

  return ret;
}

bool Adis16448::validateCrc(const std::vector<byte> &burstData) {
  if (burstData.size() != DEFAULT_BURST_LEN + 2) { return false; }

  int expected_crc = unsignedWordToInt({burstData[24], burstData[25]});
  uint16_t sampleAsWord[12];
  memset(sampleAsWord, 0, sizeof(sampleAsWord));

  int count = 0;

  for (int i = 0; i < 24; i += 2) {
    uint16_t a          = (uint16_t) Adis16448::unsignedWordToInt({burstData[i], burstData[i + 1]});
    sampleAsWord[count] = a;
    count++;
  }

  unsigned short int actual_crc = runCRC(sampleAsWord);

  return actual_crc == expected_crc;
}

unsigned short int Adis16448::runCRC(const uint16_t burstData[]) {
  unsigned char i;        // Tracks each burstData word
  unsigned char ii;       // Counter for each bit of the current burstData word
  unsigned int data;      // Holds the lower/Upper byte for CRC computation
  unsigned int crc;       // Holds the CRC value
  unsigned int lowerByte; // Lower Byte of burstData word
  unsigned int upperByte; // Upper Byte of burstData word
  unsigned int POLY;      // Divisor used during CRC computation
  POLY = 0x1021;          // Define divisor
  crc  = 0xFFFF;          // Set CRC to \f1\u8208?\f0 1 prior to beginning CRC computation
  // Compute CRC on burst data starting from XGYRO_OUT and ending with TEMP_OUT.
  // Start with the lower byte and then the upper byte of each word.
  // i.e. Compute XGYRO_OUT_LSB CRC first and then compute XGYRO_OUT_MSB CRC.
  for (i = 1; i < 12; i++) {
    upperByte = (burstData[i] >> 8) & 0xFF;
    lowerByte = (burstData[i] & 0xFF);
    data      = lowerByte; // Compute lower byte CRC first
    for (ii = 0; ii < 8; ii++, data >>= 1) {
      if ((crc & 0x0001) ^ (data & 0x0001)) crc = (crc >> 1) ^ POLY;
      else
        crc >>= 1;
    }
    data = upperByte; // Compute upper byte of CRC
    for (ii = 0; ii < 8; ii++, data >>= 1) {
      if ((crc & 0x0001) ^ (data & 0x0001)) crc = (crc >> 1) ^ POLY;
      else
        crc >>= 1;
    }
  }
  crc  = ~crc; // Compute complement of CRC\par
  data = crc;
  crc  = (crc << 8) | (data >> 8 & 0xFF); // Perform byte swap prior to returning CRC\par
  return crc;
}

void Adis16448::printImuConfig() {
  auto smpl_prd = readReg(SMPL_PRD);
  auto D        = smpl_prd[1] & 0b111111;
  LOG(I, "smpl_prd decimation rate variable D: " << +(D));
  LOG(I, "Output data rate (ODR): " << 819.2 / (1 << D) << " Hz");

  auto sens_avg = readReg(SENS_AVG);
  auto B        = sens_avg[0] & 0b111;
  LOG(I, "sens_avg filter size variable B: " << +(B));
  LOG(I, "Bartlett windows size: " << (1 << B));

  auto gyro_range = sens_avg[1] & 0b111;
  LOG(I, "sens_avg gyro range: " << 250 * (1 << (gyro_range - 1)) << " dps");
}