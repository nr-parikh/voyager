
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
 * @file    quadrotor.hpp
 * @author  nr-parikh
 * @copyright MIT license (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Header file for the classs Quadrotor.
 *
 */

#ifndef INCLUDE_VOYAGER_QUADROTOR_HPP_
#define INCLUDE_VOYAGER_QUADROTOR_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include "voyager/explore.h"
#include "voyager/planner.hpp"

class Quadrotor {
 public:
  /**
   * @brief Constructor of the class
   * @details Construct an abject of the class
   */
  Quadrotor();
  /**
   * @brief Destructor of the class
   * @details Take actions that needs to be done while destroying
   */
  ~Quadrotor();
  /**
   * @brief Explore the environment
   * @details Allow the robot to move and map the environment
   * @return void: Return nothing
   */
  auto exploreAndMap() -> void;
  /**
   * @brief Check if running
   * @details Set to true as long as the node is running
   * @return bool: Return true if running
   */
  auto isAlive() -> bool;
  /**
   * @brief Callback function
   * @details Callback function to get height of the robot from the ground
   *
   * @param height_msg: Message being received
   * @return void: Return nothing
   */
  auto heightCallback(const sensor_msgs::Range height_msg) -> void;
  /**
   * @brief Get height from ground
   * @details Function to get the height of the robot from ground
   * @return float: Height
   */
  auto getHeight() -> float;
  /**
   * @brief Callback function for service
   * @details Function callback for the service explore used by voyager
   *
   * @param request: Request of the service
   * @param resp: Response of the service
   *
   * @return bool: Return true if succeeded
   */
  auto explore(voyager::explore::Request& request,
               voyager::explore::Response& resp) -> bool;

  /**
   * @brief Get explore flag
   * @details Get the value of the explore flag set in service callback
   * @return bool: Return value of the flag
   */
  auto getExplorerFlag() -> bool;
  /**
   * @brief Stop the robot
   * @details Function to stop the robot
   * @return void: Return nothing
   */
  auto stop() -> void;

 private:
  Planner planner_;             ///< Instance of Planner class
  bool is_running_;             ///< Flag to check if running
  ros::NodeHandle nh_quad_;     ///< Nodehandle for the quadrotor class
  float height_;                ///< Height of the robot from ground
  ros::Subscriber height_sub_;  ///< Subscriber for the topic /sonar_height
  bool explore_flag_;           ///< Flag to indicate the user input
  ros::ServiceServer server_;   ///< Service server for /explore
};

#endif  // INCLUDE_VOYAGER_QUADROTOR_HPP_
