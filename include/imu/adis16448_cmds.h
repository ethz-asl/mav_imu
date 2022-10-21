//
// Created by acey on 23.08.22.
//

#ifndef MAV_IMU_INCLUDE_ADIS16448_CMDS_H_
#define MAV_IMU_INCLUDE_ADIS16448_CMDS_H_

#define CMD(x) \
  { (x), 0x00 }

#define FLASH_CNT 0x00
#define XGYRO_OUT 0x04
#define YGYRO_OUT 0x06
#define ZGYRO_OUT 0x08
#define XACCL_OUT 0x0A
#define YACCL_OUT 0x0C
#define ZACCL_OUT 0x0E
#define XMAGN_OUT 0x10
#define YMAGN_OUT 0x12
#define ZMAGN_OUT 0x14
#define BARO_OUT 0x16
#define TEMP_OUT 0x18
#define XGYRO_OFF 0x1A
#define YGYRO_OFF 0x1C
#define ZGYRO_OFF 0x1E
#define XACCL_OFF 0x20
#define YACCL_OFF 0x22
#define ZACCL_OFF 0x24
#define XMAGN_HIC 0x26
#define YMAGN_HIC 0x28
#define ZMAGN_HIC 0x2A
#define XMAGN_SIC 0x2C
#define YMAGN_SIC 0x2E
#define ZMAGN_SIC 0x30
#define GPIO_CTRL 0x32
#define MSC_CTRL 0x34
#define SMPL_PRD 0x36
#define SENS_AVG 0x38
#define SEQ_CNT 0x3A
#define DIAG_STAT 0x3C
#define GLOB_CMD 0x3E
#define ALM_MAG1 0x40
#define ALM_MAG2 0x42
#define ALM_SMPL1 0x44
#define ALM_SMPL2 0x46
#define ALM_CTRL 0x48
#define LOT_ID1 0x52
#define LOT_ID2 0x54
#define PROD_ID 0x56
#define SERIAL_NUM 0x58

#endif //MAV_IMU_INCLUDE_ADIS16448_CMDS_H_
