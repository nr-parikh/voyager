
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
#include "voyager/laser_scan.hpp"

TEST(TEST_LASERSCAN, TestLaserScanRunning) {
  // Create an instance of laser scanner
  LaserScan scanner;
  // Check if running
  EXPECT_TRUE(scanner.isAlive());
}

TEST(TEST_LASERSCAN, TestLaserScanSubscriber) {
  // Create node handle
  ros::NodeHandle nh;
  // Create a dummy publisher
  auto testPub = nh.advertise<sensor_msgs::LaserScan>("/scan", 0);
  // Wait for some time
  ros::WallDuration(1).sleep();
  // Check number of subscribers
  EXPECT_EQ(testPub.getNumSubscribers(), 1);
}

TEST(TEST_LASERSCAN, TestLaserScanCheckCollision) {
  // Create a fake laser scan message
  std::vector<float> array1(1081, 0.0);
  // Create a message
  sensor_msgs::LaserScan scanMsg;
  // Set the values
  scanMsg.angle_min = 0;
  scanMsg.angle_max = 0;
  scanMsg.angle_increment = 1;
  scanMsg.time_increment = 0;
  scanMsg.scan_time = 0;
  scanMsg.range_min = 0;
  scanMsg.range_max = 0;
  scanMsg.ranges = array1;
  scanMsg.intensities.push_back(0);

  // Create an object
  LaserScan scanner;
  // Call the call back function
  scanner.scanCallback(scanMsg);
  // Check if the flag is set to true
  EXPECT_EQ(scanner.checkCollision(), true);
  // Update the message to the other extreme condition
  std::vector<float> array2(1081, 4.0);
  scanMsg.ranges = array2;
  // Call the callback function
  scanner.scanCallback(scanMsg);
  // Check if the flag is set to false
  EXPECT_EQ(scanner.checkCollision(), false);
}
