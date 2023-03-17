#include "imu/bmi088.h"

#include <algorithm>
#include <bmi08x.h>
#include <cmath>
#include <iostream>
#include <linux/spi/spidev.h>
#include <log++.h>
#include <string>

Bmi088::Bmi088(std::string acc_path, std::string gyro_path)
    : acc_spi_driver_(std::move(acc_path)),
      gyro_spi_driver_(std::move(gyro_path)) {
  // Communication.
  dev_.intf_ptr_accel = &acc_spi_driver_;
  dev_.intf_ptr_gyro  = &gyro_spi_driver_;
  dev_.intf           = BMI08_SPI_INTF;
  dev_.variant        = BMI088_VARIANT;
  dev_.read_write_len = 32;
  dev_.read           = &(Bmi088::readReg);
  dev_.write          = &(Bmi088::writeReg);
  dev_.delay_us       = &(Bmi088::usSleep);

  // Configuration.
  dev_.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
  dev_.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
  dev_.accel_cfg.bw    = BMI08_ACCEL_BW_NORMAL;
  dev_.accel_cfg.odr   = BMI08_ACCEL_ODR_1600_HZ;

  dev_.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
  dev_.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
  dev_.gyro_cfg.bw    = BMI08_GYRO_BW_532_ODR_2000_HZ;
  dev_.gyro_cfg.odr   = BMI08_GYRO_BW_532_ODR_2000_HZ;
}

bool Bmi088::selftest() {
  LOG(I, "Performing accelerometer selftest.");
  auto acc_rslt = bmi08xa_perform_selftest(&dev_);
  printErrorCodeResults("bmi08xa_perform_selftest", acc_rslt);
  LOG(I, "Performing gyroscope selftest.");
  auto gyro_rslt = bmi08g_perform_selftest(&dev_);
  printErrorCodeResults("bmi08g_perform_selftest", gyro_rslt);
  return (acc_rslt == BMI08_OK) && (gyro_rslt == BMI08_OK);
}

bool Bmi088::setupBmiSpi() {
  // Soft reset.
  auto rslt = bmi08a_soft_reset(&dev_);
  printErrorCodeResults("bmi08a_soft_reset", rslt);
  LOG(I, rslt == BMI08_OK, "Accelerometer soft reset.");
  if (rslt != BMI08_OK) { return false; }

  // Initialize accelerometer SPI.
  rslt        = bmi08xa_init(&dev_);
  printErrorCodeResults("bmi08xa_init", rslt);
  if (rslt != BMI08_OK || dev_.accel_chip_id != BMI088_ACCEL_CHIP_ID) {
    LOG(E,
        "Failed accelerometer SPI initialization. Chip id: 0x"
            << std::hex << +dev_.accel_chip_id);
    return false;
  }
  LOG(I,
      "Accel SPI initialized. Chip id: 0x" << std::hex << +dev_.accel_chip_id);

  // Initialize gyroscope SPI.
  rslt = bmi08g_init(&dev_);
  printErrorCodeResults("bmi08g_init", rslt);
  if (rslt != BMI08_OK || dev_.gyro_chip_id != BMI08_GYRO_CHIP_ID) {
    LOG(E,
        "Failed gyroscope SPI initialization. Chip id: 0x"
            << std::hex << +dev_.gyro_chip_id);
    return false;
  }
  LOG(I, "Gyro SPI initialized. Chip id: 0x" << std::hex << +dev_.gyro_chip_id);

  // General SPI configuration.
  rslt = bmi08a_set_power_mode(&dev_);
  printErrorCodeResults("bmi08a_set_power_mode", rslt);
  if (rslt != BMI08_OK) { return false; }

  rslt = bmi08g_set_power_mode(&dev_);
  printErrorCodeResults("bmi08g_set_power_mode", rslt);
  if (rslt != BMI08_OK) { return false; }

  // TODO(rikba): Not sure if this step is required.
  rslt = bmi08a_load_config_file(&dev_);
  printErrorCodeResults("bmi08a_load_config_file", rslt);
  if (rslt != BMI08_OK) { return false; }

  rslt = bmi08xa_set_meas_conf(&dev_);
  printErrorCodeResults("bmi08xa_set_meas_conf", rslt);
  if (rslt != BMI08_OK) { return false; }

  return true;
}

bool Bmi088::init() {
  // Create SPI drivers.
  if (!acc_spi_driver_.open()) {
    LOG(E, "Accelerometer open failed: " << strerror(errno));
    return false;
  }

  if (!gyro_spi_driver_.open()) {
    LOG(E, "Gyroscope open failed: " << strerror(errno));
    return false;
  }

  if (!acc_spi_driver_.setMode(SPI_MODE_3)) {
    LOG(E, "Accelerometer setmode failed");
    return false;
  }

  if (!gyro_spi_driver_.setMode(SPI_MODE_3)) {
    LOG(E, "Gyroscope setmode failed");
    return false;
  }

  // Reset SPI communication.
  if (!setupBmiSpi()) return false;

  // Self test.
  if (!selftest()) { return false; }

  // Another reset recommended.
  if (!setupBmiSpi()) return false;

  // Re-configure data acquisition and filtering.
  bmi08_data_sync_cfg sync_cfg;
  // TODO(rikba): Expose sync_cfg setting to user.
  sync_cfg.mode = BMI08_ACCEL_DATA_SYNC_MODE_400HZ;
  auto rslt          = bmi08xa_configure_data_synchronization(sync_cfg, &dev_);
  printErrorCodeResults("bmi08a_configure_data_synchronization", rslt);
  LOG(I, "Configured IMU data synchronization.");

  /*set accel interrupt pin configuration*/
  /*configure host data ready interrupt */
  bmi08_int_cfg int_config;
  int_config.accel_int_config_1.int_channel = BMI08_INT_CHANNEL_1;
  int_config.accel_int_config_1.int_type    = BMI08_ACCEL_SYNC_INPUT;
  int_config.accel_int_config_1.int_pin_cfg.output_mode =
      BMI08_INT_MODE_PUSH_PULL;
  int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
  int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

  /*configure Accel syncronization input interrupt pin */
  int_config.accel_int_config_2.int_channel = BMI08_INT_CHANNEL_2;
  int_config.accel_int_config_2.int_type    = BMI08_ACCEL_INT_SYNC_DATA_RDY;
  int_config.accel_int_config_2.int_pin_cfg.output_mode =
      BMI08_INT_MODE_PUSH_PULL;
  int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
  int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

  /*set gyro interrupt pin configuration*/
  int_config.gyro_int_config_1.int_channel = BMI08_INT_CHANNEL_3;
  int_config.gyro_int_config_1.int_type    = BMI08_GYRO_INT_DATA_RDY;
  int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
  int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
  int_config.gyro_int_config_1.int_pin_cfg.output_mode =
      BMI08_INT_MODE_PUSH_PULL;

  int_config.gyro_int_config_2.int_channel = BMI08_INT_CHANNEL_4;
  int_config.gyro_int_config_2.int_type    = BMI08_GYRO_INT_DATA_RDY;
  int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08_DISABLE;
  int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
  int_config.gyro_int_config_2.int_pin_cfg.output_mode =
      BMI08_INT_MODE_PUSH_PULL;

  /* Enable synchronization interrupt pin */
  rslt = bmi08a_set_data_sync_int_config(&int_config, &dev_);
  printErrorCodeResults("bmi08a_set_data_sync_int_config", rslt);
  return true;
}

bool Bmi088::close() {
  return acc_spi_driver_.close() && gyro_spi_driver_.close();
}

int Bmi088::getRaw(std::vector<byte> cmd) {
  LOG(E, "Bmi088 getRaw not implemented.");
}

int8_t Bmi088::readReg(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr) {
  auto res = static_cast<SpiDriver *>(intf_ptr)->xfer({reg_addr}, len,
                                                      spi_transfer_speed_hz_);
  std::copy(res.begin(), res.end(), reg_data);
  return res.empty() ? BMI08_E_COM_FAIL : BMI08_OK;
}

int8_t Bmi088::writeReg(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr) {
  std::vector<uint8_t> req = {reg_addr};
  std::copy(&reg_data[0], &reg_data[len], std::back_inserter(req));
  // TODO(rikba): Implement IOCT error.
  static_cast<SpiDriver *>(intf_ptr)->xfer(req, 0, spi_transfer_speed_hz_);
  return BMI08_OK;
}

std::optional<vec3<double>> Bmi088::getGyro() {
  bmi08_sensor_data gyro;
  auto rslt = bmi08g_get_data(&gyro, &dev_);
  printErrorCodeResults("bmi08g_get_data", rslt);

  if (rslt == BMI08_OK) {
    return vec3<double>(
        {lsbToRps(gyro.x, dev_.gyro_cfg.range, BMI08_16_BIT_RESOLUTION),
         lsbToRps(gyro.y, dev_.gyro_cfg.range, BMI08_16_BIT_RESOLUTION),
         lsbToRps(gyro.z, dev_.gyro_cfg.range, BMI08_16_BIT_RESOLUTION)});
  } else {
    return std::nullopt;
  }
}

std::optional<vec3<double>> Bmi088::getAcceleration() {
  bmi08_sensor_data acc;
  auto rslt = bmi08a_get_data(&acc, &dev_);
  printErrorCodeResults("bmi08a_get_data", rslt);

  if (rslt == BMI08_OK) {
    return vec3<double>(
        {lsbToMps2(acc.x, dev_.accel_cfg.range, BMI08_16_BIT_RESOLUTION),
         lsbToMps2(acc.y, dev_.accel_cfg.range, BMI08_16_BIT_RESOLUTION),
         lsbToMps2(acc.z, dev_.accel_cfg.range, BMI08_16_BIT_RESOLUTION)});
  } else {
    return std::nullopt;
  }
}

void Bmi088::usSleep(uint32_t period, void *intf_ptr) { usleep(period); }

void Bmi088::printErrorCodeResults(const std::string &api_name, int8_t rslt) {
  if (rslt != BMI08_OK) {
    LOG(E, api_name.c_str() << "\t");
    if (rslt == BMI08_E_NULL_PTR) {
      LOG(E, "Error [" << int(rslt) << "] : Null pointer\r\n");
    } else if (rslt == BMI08_E_COM_FAIL) {
      LOG(E, "Error [" << int(rslt) << "] : Communication failure\r\n");
    } else if (rslt == BMI08_E_DEV_NOT_FOUND) {
      LOG(E, "Error [" << int(rslt) << "] : Device not found\r\n");
    } else if (rslt == BMI08_E_OUT_OF_RANGE) {
      LOG(E, "Error [" << int(rslt) << "] : Out of Range\r\n");
    } else if (rslt == BMI08_E_INVALID_INPUT) {
      LOG(E, "Error [" << int(rslt) << "] : Invalid input\r\n");
    } else if (rslt == BMI08_E_CONFIG_STREAM_ERROR) {
      LOG(E, "Error [" << int(rslt) << "] : Config stream error\r\n");
    } else if (rslt == BMI08_E_RD_WR_LENGTH_INVALID) {
      LOG(E, "Error [" << int(rslt) << "] : Invalid Read write length\r\n");
    } else if (rslt == BMI08_E_INVALID_CONFIG) {
      LOG(E, "Error [" << int(rslt) << "] : Invalid config\r\n");
    } else if (rslt == BMI08_E_FEATURE_NOT_SUPPORTED) {
      LOG(E, "Error [" << int(rslt) << "] : Feature not supported\r\n");
    } else if (rslt == BMI08_W_FIFO_EMPTY) {
      printf("Warning [%d] : FIFO empty\r\n");
    } else {
      LOG(E, "Error [" << int(rslt) << "] : Unknown error code\r\n");
    }
  }
}

void Bmi088::printGyroBw() {
  uint16_t bw = 0xFFFF;
  if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_532_ODR_2000_HZ) {
    bw = 532;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_230_ODR_2000_HZ) {
    bw = 230;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_116_ODR_1000_HZ) {
    bw = 116;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_47_ODR_400_HZ) {
    bw = 47;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_23_ODR_200_HZ) {
    bw = 23;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_12_ODR_100_HZ) {
    bw = 12;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_64_ODR_200_HZ) {
    bw = 64;
  } else if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_32_ODR_100_HZ) {
    bw = 32;
  }

  if (bw != 0xFFFF) {
    LOG(I, "gyro_cfg.bw: " << bw << " Hz");
  } else {
    LOG(E, "Error printing gyro bandwidth.");
  }
}

void Bmi088::printGyroOdr() {
  uint16_t odr = 0xFFFF;
  if (dev_.gyro_cfg.bw == BMI08_GYRO_BW_532_ODR_2000_HZ) {
    odr = 2000;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_230_ODR_2000_HZ) {
    odr = 2000;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_116_ODR_1000_HZ) {
    odr = 1000;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_47_ODR_400_HZ) {
    odr = 400;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_23_ODR_200_HZ) {
    odr = 200;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_12_ODR_100_HZ) {
    odr = 100;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_64_ODR_200_HZ) {
    odr = 200;
  } else if (dev_.gyro_cfg.odr == BMI08_GYRO_BW_32_ODR_100_HZ) {
    odr = 100;
  }

  if (odr != 0xFFFF) {
    LOG(I, "gyro_cfg.odr: " << odr << " Hz");
  } else {
    LOG(E, "Error printing gyro ODR.");
  }
}

float Bmi088::lsbToMps2(int16_t val, int8_t g_range, uint8_t bit_width) {

  float half_scale = (float) ((std::pow(2.0f, (double) bit_width) / 2.0f));

  return (g_ * val * computeAccRange(g_range)) / half_scale;
}

float Bmi088::lsbToRps(int16_t val, uint8_t dps_range, uint8_t bit_width) {

  float half_scale = (float) ((std::pow(2.0f, (double) bit_width) / 2.0f));

  uint16_t dps_res = dps_range_max_ / (1 << dps_range);
  return (computeGyroRange(dps_range) / (half_scale)) * (val) * (M_PI / 180.);
}

uint8_t Bmi088::computeAccRange(uint8_t accel_cfg_range) {
  return g_range_min_ * (1 << (accel_cfg_range - BMI088_ACCEL_RANGE_3G));
}

uint16_t Bmi088::computeGyroRange(uint16_t gyro_cfg_range) {
  return dps_range_max_ / (1 << (gyro_cfg_range - BMI08_GYRO_RANGE_2000_DPS));
}

uint8_t Bmi088::computeAccBw(uint8_t accel_cfg_bw) {
  return acc_bw_osr_max_ / (1 << (accel_cfg_bw - BMI08_ACCEL_BW_OSR4));
}

uint16_t Bmi088::computeAccOdr(uint16_t accel_cfg_odr) {
  return acc_odr_min_ * (1 << (accel_cfg_odr - BMI08_ACCEL_ODR_12_5_HZ));
}

void Bmi088::printImuConfig() {
  int8_t rslt = bmi08g_get_meas_conf(&dev_);
  printErrorCodeResults("bmi08g_get_meas_conf", rslt);

  rslt = bmi08a_get_meas_conf(&dev_);
  printErrorCodeResults("bmi08a_get_meas_conf", rslt);

  LOG(I,
      "accel_cfg.range: " << +computeAccRange(dev_.accel_cfg.range)
                          << " m/s^2");
  LOG(I, "accel_cfg.bw (OSR):" << +computeAccBw(dev_.accel_cfg.bw));
  LOG(I, "accel_cfg.odr: " << computeAccOdr(dev_.accel_cfg.odr) << " Hz");
  LOG(I, "gyro_cfg.range: " << computeGyroRange(dev_.gyro_cfg.range) << " dps");
  printGyroBw();
  printGyroOdr();
}

ImuBurstResult Bmi088::burst() {
  struct ImuBurstResult ret {};
  bmi08_sensor_data acc, gyro;

  // TODO(rikba): This is not really burst but rather returning one after the other. Implement actual burst.
  // TODO(rikba): Temperature
  auto rslt = bmi08a_get_synchronized_data(&acc, &gyro, &dev_);
  printErrorCodeResults("bmi08a_get_synchronized_data", rslt);

  if (rslt == BMI08_OK) {
    ret.acceleration = vec3<double>(
        {lsbToMps2(acc.x, dev_.accel_cfg.range, BMI08_16_BIT_RESOLUTION),
         lsbToMps2(acc.y, dev_.accel_cfg.range, BMI08_16_BIT_RESOLUTION),
         lsbToMps2(acc.z, dev_.accel_cfg.range, BMI08_16_BIT_RESOLUTION)});
    ret.gyro = vec3<double>(
        {lsbToRps(gyro.x, dev_.gyro_cfg.range, BMI08_16_BIT_RESOLUTION),
         lsbToRps(gyro.y, dev_.gyro_cfg.range, BMI08_16_BIT_RESOLUTION),
         lsbToRps(gyro.z, dev_.gyro_cfg.range, BMI08_16_BIT_RESOLUTION)});
  }

  return ret;
}