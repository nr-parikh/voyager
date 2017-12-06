
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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "voyager/planner.hpp"

class Quadrotor
{
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
	auto exploreAndMap()->void;
	/**
	 * @brief Check if running 
	 * @details Set to true as long as the node is running  
	 * @return bool: Return true if running 
	 */	
	auto isAlive()->bool;
private:
	Planner planner_; ///< Instance of Planner class 
	bool is_running_; ///< Flag to check if running 
	ros::NodeHandle nh_quad_; ///< Nodehandle for the quadrotor class 
	
};

#endif // INCLUDE_VOYAGER_PLANNER_HPP_