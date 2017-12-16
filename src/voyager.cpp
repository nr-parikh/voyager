
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
 * @file    voyager.cpp
 * @author  nr-parikh
 * @copyright MIT license (c) 2017 Neel Parikh
 *
 * @brief DESCRIPTION
 * Main file for the package voyager
 *
 */

#include <hector_uav_msgs/EnableMotors.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "voyager/explore.h"
#include "voyager/quadrotor.hpp"

int main(int argc, char** argv) {
  // Initialize the node
  ros::init(argc, argv, "voyager_node");
  // Create a nodehandle
  ros::NodeHandle nh;
  // Inform the user
  ROS_INFO_STREAM("Starting Voyager node...");
  // Create a client for service /explore
  ros::ServiceClient exploreClient =
      nh.serviceClient<voyager::explore>("explore");
  // Create a client for service /enable_motors
  ros::ServiceClient motorsClient =
      nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
  // Create a service instance
  hector_uav_msgs::EnableMotors srv;
  // Set the request
  srv.request.enable = true;
  // Call the service
  motorsClient.call(srv);
  // Wait for service to advertise
  if (ros::service::waitForService("explore", 1000)) {
    ROS_INFO_STREAM("The service is available!");
  }
  // Create an instance of the quadrotor
  Quadrotor quad;
  // Check rosmaster
  while (ros::ok()) {
    // Check the user input
    if (quad.getExplorerFlag()) {
      // Explore the map
      quad.exploreAndMap();
    } else {
      // Stay at the same place
      quad.stop();
    }
    // Spin in callbacks
    ros::spinOnce();
  }
  // Disable motors
  srv.request.enable = false;
  // Call the service
  motorsClient.call(srv);

  return 0;
}
