
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
 * @file    planner.hpp
 * @author  nr-parikh
 * @copyright MIT license (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Header file for the classs Planner.
 *
 */
 
#ifndef INCLUDE_VOYAGER_PLANNER_HPP_
#define INCLUDE_VOYAGER_PLANNER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "voyager/laser_scan.hpp"

class Planner
{
public:
	/**
	 * @brief Constructor of the class 
	 * @details Construct an abject of the class 
	 */
	Planner();
	/**
	 * @brief Destructor of the class 
	 * @details Take actions that needs to be done while destroying
	 */
	~Planner();
	/**
	 * @brief Take the action
	 * @details Move robot according if there are no obstacles 
	 * @return void: Return nothing 
	 */
	auto takeAction()->void;
	/**
	 * @brief Check if running 
	 * @details Set to true as long as the node is running  
	 * @return bool: Return true if running 
	 */	
	auto isAlive()->bool;
private:
	LaserScan laser_scan_; ///< Instance of LaserScan class 
	bool is_running_; ///< Flag to check if running 
	ros::NodeHandle nh_planner_; ///< Nodehandle for the planner class 
	geometry_msgs::Twist vel_msg_; ///< Message over which data will be published 
	ros::Publisher vel_pub_; ///< Publisher for publishing velocity
	
};

#endif // INCLUDE_VOYAGER_PLANNER_HPP_