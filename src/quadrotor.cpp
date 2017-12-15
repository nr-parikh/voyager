
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
 * @file    quadrotor.cpp
 * @author  nr-parikh
 * @copyright MIT license (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Implementation of the class Quadrotor
 *
 */

#include "voyager/quadrotor.hpp"
#include <cmath>

Quadrotor::Quadrotor() {
  // Set the running flag
  is_running_ = true;
  // Inform the user
  ROS_INFO_STREAM("Starting the Quadrotor...");
  // Create a subscriber
  height_sub_ = nh_quad_.subscribe<sensor_msgs::Range>(
      "/sonar_height", 1000, &Quadrotor::heightCallback, this);
  // Set the height to a high variable
  height_ = std::numeric_limits<float>::quiet_NaN();
  // Advertise the service
  server_ = nh_quad_.advertiseService("explore", &Quadrotor::explore, this);
}

Quadrotor::~Quadrotor() {
  // Stop the planner
  planner_.stop();
}

auto Quadrotor::exploreAndMap() -> void {
  // Check the height of the robot
  if (height_ > 1) {
    // Go down
    planner_.setVelocity(-1);
    // Check the height of the robot
  } else if (height_ < 1 && height_ > 0) {
    // Go up
    planner_.setVelocity(1);
  } else {
    // Stay at the same place
    planner_.setVelocity(0.0);
  }
  // Explore the map
  planner_.takeAction();
}

auto Quadrotor::isAlive() -> bool {
  // Return the running flag
  return is_running_;
}

auto Quadrotor::heightCallback(const sensor_msgs::Range height_msg) -> void {
  // update the height of the robot
  height_ = height_msg.range;
}

auto Quadrotor::getHeight() -> float {
  // Return the height of the robot
  return height_;
}

auto Quadrotor::explore(voyager::explore::Request& request,
                        voyager::explore::Response& resp) -> bool {
  // Get the service request
  explore_flag_ = request.explore_flag;
  // Update the service response
  resp.response = true;
}

auto Quadrotor::getExplorerFlag() -> bool {
  // Get the value of explore flag
  return explore_flag_;
}

auto Quadrotor::stop() -> void {
  // Check the height of the robot
  while (height_ > 0.2) {
    // Stay on ground
    planner_.setVelocity(-1);
    // Spin in callbacks
    ros::spinOnce();
  }
}
