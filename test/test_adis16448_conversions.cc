//
// Created by 4c3y (acey) on 09.09.22.
//

#include <gtest/gtest.h>
#include "imu/adis16448.h"


TEST(abc, def) {

  unsigned char byte1 = 255;
  unsigned char byte2 = 255;
  int c = Adis16448::signedWordToInt({byte1, byte2});

  ASSERT_EQ(c, -1);
}

TEST(abc, defg) {

  unsigned char byte1 = 1;
  unsigned char byte2 = 5;
  int c = Adis16448::signedWordToInt({byte1, byte2});

  ASSERT_EQ(c, 261);
}