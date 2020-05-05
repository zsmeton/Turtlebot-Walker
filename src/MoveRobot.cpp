/**
 * Copyright (c) 2018, Adarsh Jagan Sathyamoorthy
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file       MoveRobot.cpp
 * @author     Adarsh Jagan Sathyamoorthy
 * @copyright  3-clause BSD
 * @brief MoveRobot class' function definitions
 * Defines the methods for the MoveRobot class.
 */

#include "MoveRobot.hpp"

MoveRobot::MoveRobot() {
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;
  pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",
                                           100);
  sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 50, &Sensor::scanCb,
                                             &scan);
  pub.publish(vel);
}

MoveRobot::~MoveRobot() {
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;
  pub.publish(vel);
}

void MoveRobot::publishVelocity() {
  if (scan.returnCollisionFlag() == true) {
    float collAngle = scan.returnCollisionAngle();
    // Rotate CW and backwards if collision is detected.
    ROS_INFO_STREAM("Collision at angle " << collAngle);
    if(collAngle < M_PI/2.0){
      // Collision front right
      vel.linear.x = -0.3;
      vel.angular.z = -1.0;
    }else if (collAngle >= M_PI/2.0 && collAngle < M_PI) {
      // Collision on front left
      vel.linear.x = -0.3;
      vel.angular.z = 1.0;
    }else if (collAngle >= M_PI && collAngle < 3.0*M_PI/2.0){
      // Collision on rear left
      vel.linear.x = 0.3;
      vel.angular.z = -1.0;
    }else{
      // Collision on rear right
      vel.linear.x = 0.3;
      vel.angular.z = 1.0;
    }
  
  } else {
    sleep(1);
    // Move forward if no collision is detected
    vel.linear.x = -0.5;
    // With some random twist
    vel.angular.z = 0.0;
  }
  pub.publish(vel);
}
