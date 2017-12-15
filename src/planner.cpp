
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
 * @file    planner.cpp
 * @author  nr-parikh
 * @copyright MIT license (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Implementation of the class Planner
 *
 */

#include "voyager/planner.hpp"
#include <stdlib.h>

Planner::Planner() {
  // Set running flag
  is_running_ = true;
  // Inform planner is beign initiated
  ROS_INFO_STREAM("Initializing the Planner...");
  // Set the velocities to 0
  vel_pub_ = nh_planner_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  vel_msg_.linear.x = 0;
  vel_msg_.linear.y = 0;
  vel_msg_.linear.z = 0;
  vel_msg_.angular.x = 0;
  vel_msg_.angular.y = 0;
  vel_msg_.angular.z = 0;
  // Publish velocities
  vel_pub_.publish(vel_msg_);
}

Planner::~Planner() {
  // Set running flag to false
  is_running_ = false;
  // Stop the planner
  stop();
}

auto Planner::takeAction() -> void {
  // Check the value of obstacle flag
  if (!laser_scan_.checkCollision()) {
    // Go forward
    vel_msg_.linear.x = 0.5;
    vel_msg_.angular.z = 0.0;
    // Publish velocity
    vel_pub_.publish(vel_msg_);
  } else {
    // Rotate to avoid obstacle
    vel_msg_.linear.x = 0.0;
    vel_msg_.angular.z = -0.3;
    // Publish velocity
    vel_pub_.publish(vel_msg_);
  }
}

auto Planner::isAlive() -> bool {
  // Return the running flag
  return is_running_;
}

auto Planner::setVelocity(float velocity) -> void {
  // Set the vertical velocity
  vel_msg_.linear.z = velocity;
  // Publish the velocity
  vel_pub_.publish(vel_msg_);
}

auto Planner::stop() -> void {
  // Set the velocities to 0
  vel_msg_.linear.x = 0;
  vel_msg_.linear.y = 0;
  vel_msg_.linear.z = 0;
  vel_msg_.angular.x = 0;
  vel_msg_.angular.y = 0;
  vel_msg_.angular.z = 0;
  // Publish the velocity
  vel_pub_.publish(vel_msg_);
}
