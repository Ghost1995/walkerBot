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
 * @file walkerbot.cpp
 * @author Ashwin Goyal
 * @copyright BSD 3-Clause License
 *
 * @brief This is the implementation of the walkerbot class
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "walkerbot/walkerbot.hpp"

// This is the constructor for the class
walkerbot::walkerbot() {
    obsDetected = false;
}

// This is the first method of the class. It is a callback function for
// subscribing to the laser scan data.
void walkerbot::readLaserSensor(const sensor_msgs::LaserScan::ConstPtr& msg) {
    for (auto i = 0; i < msg->ranges.size(); ++i) {
        ROS_DEBUG_STREAM("Range is: " << msg->ranges[i]);
        if (msg->ranges[i] < 2) {
            obsDetected = true;
            ROS_DEBUG_STREAM("Range is less than 0.7: " << msg->ranges[i]);
            return;
        }
    }
    obsDetected = false;
    ROS_DEBUG_STREAM("No obstacles in the defined range.");
}

// This is the second method of the class. It returns whether the robot detects
// an obstacle or not.
bool walkerbot::isFree() {
  return !obsDetected;
}

// This is the destructor for the class
walkerbot::~walkerbot() {
    ROS_INFO_STREAM("The walkerbot node is shutting down.");
}
