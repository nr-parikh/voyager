
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
  ros::init(argc, argv, "voyager_node");

  ros::NodeHandle nh;

  ROS_INFO_STREAM("Starting Voyager node...");

  ros::ServiceClient exploreClient =
      nh.serviceClient<voyager::explore>("explore");

  ros::ServiceClient motorsClient =
      nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");

  hector_uav_msgs::EnableMotors srv;
  srv.request.enable = true;

  if (motorsClient.call(srv)) {
    ROS_INFO_STREAM("Enabling motors...");
  }

  if (ros::service::waitForService("explore", 1000)) {
    ROS_INFO_STREAM("The service is available!");
  }

  Quadrotor quad;

  while (ros::ok()) {
    if (quad.getExplorerFlag()) {
      quad.exploreAndMap();
    } else {
      quad.stop();
    }
    ros::spinOnce();
  }
  srv.request.enable = false;
  if (motorsClient.call(srv)) {
    ROS_INFO_STREAM("Disabling motors...");
  }
  return 0;
}
