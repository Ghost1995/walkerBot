/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2018, Ashwin Goyal
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIALDAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file main.cpp
 * @author Ashwin Goyal
 * @copyright BSD 3-Clause License
 *
 * @brief It is the implementation of a simple walker algorithm. This algorithm
 *        makes the robot move forward until it detects an obstacle. Then the
 *        robot rotates at its position to find free space. Once free space is
 *        found, the robot again moves forward until an obstacle is detected.
 */

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "walkerbot/walkerbot.hpp"

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "walkerbot");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Initiate an instance of walkerbot class
  walkerbot bot;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  auto laserSensor = n.subscribe<sensor_msgs::LaserScan>("/scan", 50,
                                            &walkerbot::readLaserSensor, &bot);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  auto vel = n.advertise<geometry_msgs::Twist>
                                      ("/mobile_base/commands/velocity", 1000);

  // Set frequency of publishing the data
  ros::Rate loop_rate(10);

  // Initiaize the velocity message (default is set as 0)
  geometry_msgs::Twist msg;

  // Run the following commands only if the ros node is working
  int count = 0;
  while (ros::ok()) {
    // If the robot does not see an obstacle
    if (bot.isFree()) {
      // Move forward
      msg.linear.x = 0.5;
      // Ensure that the robot is not rotating
      msg.angular.z = 0.0;
      // Print this statement on the console
      ROS_INFO_STREAM("Moving Forward");
      // Change the count to be greater than 0
      count += 1;
    } else { // If the robot does see an obstacle
      // Rotate at its position
      msg.angular.z = 0.5;
      // Ensure that the robot is not moving forward
      msg.linear.x = 0.0;
      // Print this statement on the console as a warning
      ROS_WARN_COND(count > 0, "Obstacle Present!");
      ROS_INFO_STREAM("Turning to Avoid the Obstacle.");
      // Warning printed. Don't print it again till the next obstacle is found.
      count = 0;
    }

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    vel.publish(msg);

    // Now, calling all the callbacks waiting to be called.
    ros::spinOnce();

    // Waiting to ensure that the publish frequency is achieved.
    loop_rate.sleep();
  }

  // Return an int to avoid an error.
  return 0;
}
