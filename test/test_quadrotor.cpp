
// MIT License

// Copyright (c) 2017 Neel Parikh

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file    test_laser_scan.cpp
 * @author  nr-parikh
 * @copyright MIT license (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Unit tests for the class LaserScan
 *
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "voyager/quadrotor.hpp"

TEST(TEST_QUADROTOR, TestQuadrotorRunning) {
  Quadrotor quad;

  EXPECT_TRUE(quad.isAlive());
}

TEST(TEST_QUADROTOR, TestQuadrotorHeightSubscriber) {
  ros::NodeHandle nh;
  Quadrotor quad;

  ros::Publisher testPub = nh.advertise<sensor_msgs::Range>("/sonar_height", 0);
  ros::WallDuration(1).sleep();
  EXPECT_EQ(testPub.getNumSubscribers(), 2);
}

TEST(TEST_QUADROTOR, TestQuadrotorGetHeight) {
  Quadrotor quad;

  sensor_msgs::Range rangeMsg;

  rangeMsg.radiation_type = 0;
  rangeMsg.field_of_view = 0.0;
  rangeMsg.min_range = 0.0;
  rangeMsg.max_range = 0.0;
  rangeMsg.range = 0.0;

  quad.heightCallback(rangeMsg);

  EXPECT_EQ(quad.getHeight(), 0.0);
}