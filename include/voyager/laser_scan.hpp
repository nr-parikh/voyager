
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
 * @file    laser_scan.hpp
 * @author  nr-parikh
 * @copyright MIT license (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Header file for the class LaserScan.
 *
 */

#ifndef INCLUDE_VOYAGER_LASER_SCAN_
#define INCLUDE_VOYAGER_LASER_SCAN_ 

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserScan {
 public:
 /**
  * @brief Constructor of the class 
  * @details Initialize node handle as well as set is_running_ flag to true
  */
  LaserScan();
  /**
   * @brief Destructor of the class
   * @details Write actions that needs to be done when destroying the object
   */
  ~LaserScan();
  /**
   * @brief Callback function
   * @details Callback function for LaserScan messages. Check if there is any collision. 
   * 
   * @param scan_msg Incoming message of type sensor_msgs/LaserScan 
   */
  auto scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)->void;
  /**
   * @brief Check if running 
   * @details Set to true as long as the node is running  
   * @return bool: Return true if running 
   */ 
  auto isAlive()->bool;
 private:
    ros::NodeHandle nh_laser_scan_; ///< Node handle for this class 
    bool is_running_; ///< Flag to indicate if the class is running 
    bool obstacle_flag_; ///< Flag to indicate if obstacle is detected 
    ros::Subscriber scan_sub_; ///< Subscriber that can be used to listen to LaserScan messages
};

#endif