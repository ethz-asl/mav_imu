//
// Created by acey on 01.09.22.
//
#include <gtest/gtest.h>

#include <imu/imu_interface.h>
#include "log++/log++.h"


TEST(vec3_toString, basic) {
  vec3<int> a{8, 4, 2};

  ASSERT_EQ(a.toString(), "X: 8 Y: 4 Z: 2");
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

TEST(vec3_overload, division_assignment) {
  vec3<int> a{8, 4, 2};

   a /= 2;
  ASSERT_EQ(a.x, 4);
  ASSERT_EQ(a.y, 2);
  ASSERT_EQ(a.z, 1);
}

TEST(vec3_overload, multiplication) {
  vec3<int> a{4, 2, 1};
  vec3<int> b = a * 2;
  ASSERT_EQ(b.x, 8);
  ASSERT_EQ(b.y, 4);
  ASSERT_EQ(b.z, 2);
}

TEST(vec3_overload, multiplication_assignment) {
  //TODO fix
  vec3<int> a{4, 2, 1};
  //a *= 2;
  ASSERT_EQ(a.x, 8);
  ASSERT_EQ(a.y, 4);
  ASSERT_EQ(a.z, 2);
}