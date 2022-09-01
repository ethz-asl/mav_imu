//
// Created by acey on 01.09.22.
//
#include <gtest/gtest.h>

#include <imu/imu_interface.h>
#include "log++/log++.h"

using namespace testing::internal;
TEST(vec3_toString, basic) {
  vec3<int> a{8, 4, 2};

  std::string out = a.toString();
  ASSERT_EQ(out, "X: 8 Y: 4 Z: 2");
}

TEST(vec3_overload, division) {
  vec3<int> a{8, 4, 2};

  vec3<int> b = a / 2;

  ASSERT_EQ(a.x, 8);
  ASSERT_EQ(a.y, 4);
  ASSERT_EQ(a.z, 2);

  ASSERT_EQ(b.x, 4);
  ASSERT_EQ(b.y, 2);
  ASSERT_EQ(b.z, 1);
}