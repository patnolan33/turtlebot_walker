/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, patnolan33
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * @file main.cpp
 * @brief Walker entry point
 * @details This file is used as the entry point for the ROS walker class
 * @author Patrick Nolan (patnolan33)
 * @copyright BSD
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "walker.hpp"

int main(int argc, char **argv) {
  // Initialize ROS and name our node "talker"
  ros::init(argc, argv, "walker");

  // Handle for the process node. Will handle initialization and
  //   cleanup of the node
  ros::NodeHandle n;

  // Walker object
  Walker walker;

  // Subscribe to the "scan" topic to listen for any
  //   messages published on that topic.
  // Set the buffer to 500 messages
  // Set the callback to the chatterCallback method
  ros::Subscriber sub = n.subscribe < sensor_msgs::LaserScan
      > ("/scan", 500, &Walker::laserCallback, &walker);

  // Publish the "velocity" topic to the turtlebot
  ros::Publisher velocityPub = n.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 1000);

  // Set up the publisher rate to 10 Hz
  ros::Rate loop_rate(10);

  // Initialize the twist message that we will command turtlebot with
  geometry_msgs::Twist msg;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  // While running, check if a collision is about to occur.
  //  -If so, stop moving forward and rotate about the z-axis
  //  -If not, stop rotating and move forward
  while (ros::ok()) {
    if (walker.collisionDetected()) {
      // Set linear velocity to zero
      msg.linear.x = 0.0;
      // Set turn rate about the z-axis
      msg.angular.z = 1.0;
    } else {
      // Set turn rate to zero
      msg.angular.z = 0.0;
      // Move forward slowly
      msg.linear.x = 0.1;
    }

    // Publish the twist message to anyone listening
    velocityPub.publish(msg);

    // "Spin" a callback in case we set up any callbacks
    ros::spinOnce();

    // Sleep for the remaining time until we hit our 10 Hz rate
    loop_rate.sleep();
  }

  return 0;
}
