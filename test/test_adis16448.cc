//
// Created by 4c3y (acey) on 09.09.22.
//

#include <gtest/gtest.h>
#include "imu/adis16448.h"

TEST(adis16448_conversions, signedWordToInt_Overflow) {

  unsigned char byte1 = 255;
  unsigned char byte2 = 255;
  int c = Adis16448::signedWordToInt({byte1, byte2});

  EXPECT_EQ(c, -1);
}

TEST(adis16448_conversions, signedWordToInt_Basic) {

  unsigned char byte1 = 1;
  unsigned char byte2 = 5;
  int c = Adis16448::signedWordToInt({byte1, byte2});

  EXPECT_EQ(c, 261);
}

TEST(adis16448_conversions, unsignedWordToInt_Basic) {

  int res = Adis16448::unsignedWordToInt({0xe1, 0xf3});
  EXPECT_EQ(0xE1F3, res);
}

TEST(adis16448, burst_with_crc_toggle) {
  Adis16448 adis_16448{"/dev/spidev0.1"};
  adis_16448.init({{"crc", "true"}});
  adis_16448.burst();
  usleep(1e3);
  adis_16448.burst();
  usleep(1e3);
  adis_16448.burst();
  usleep(1e3);
  adis_16448.burst();
  usleep(1e3);

  adis_16448.setBurstCRCEnabled(false);
  adis_16448.burst();
}

TEST(adis16448, crc) {
  std::vector<byte> sample =
      {0x00, 0x80, 0x00, 0x03, 0xff, 0xfe, 0xff, 0xe6, 0xff, 0xed, 0x00, 0x03, 0x04, 0xde, 0x00, 0xfb, 0x03, 0x1b, 0xf7,
       0x11, 0xbd, 0x81, 0xff, 0xc2, 0xe1, 0xf3};

  EXPECT_TRUE(Adis16448::validateCrc(sample));
}