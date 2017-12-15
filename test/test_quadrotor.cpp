
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
  // Create an instance of quadrotor
  Quadrotor quad;
  // Check if running
  EXPECT_TRUE(quad.isAlive());
}

TEST(TEST_QUADROTOR, TestQuadrotorHeightSubscriber) {
  // Create node handle
  ros::NodeHandle nh;
  // Create an instance of quadrotor
  Quadrotor quad;
  // Create a dummy publisher
  ros::Publisher testPub = nh.advertise<sensor_msgs::Range>("/sonar_height", 0);
  // Wait for some time s
  ros::WallDuration(1).sleep();
  // Check the number of subscribers
  EXPECT_EQ(testPub.getNumSubscribers(), 2);
}

TEST(TEST_QUADROTOR, TestQuadrotorGetHeight) {
  // Create an instance of quadrotor
  Quadrotor quad;
  // Create a dummy reading
  sensor_msgs::Range rangeMsg;
  // Set the values
  rangeMsg.radiation_type = 0;
  rangeMsg.field_of_view = 0.0;
  rangeMsg.min_range = 0.0;
  rangeMsg.max_range = 0.0;
  rangeMsg.range = 0.0;
  // Call the callback function
  quad.heightCallback(rangeMsg);
  // Check the value of height
  EXPECT_EQ(quad.getHeight(), 0.0);
}
