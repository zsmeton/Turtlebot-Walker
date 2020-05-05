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
 * @file       Sensor.cpp
 * @author     Adarsh Jagan Sathyamoorthy
 * @copyright  3-clause BSD
 * @brief Sensor class function definitions
 */

#include "Sensor.hpp"

Sensor::Sensor() {
  collisionFlag = false;
  collisionAngle = 0.0;
  minAngle = 0.0;
  maxAngle = 2*M_PI;
  collDist = 0.3;
}

Sensor::~Sensor() {

}



void Sensor::scanCb(const sensor_msgs::LaserScan::ConstPtr& scan) {
  std::vector<bool> collisions;

  // Collision Detect
  bool coll = false;
  float angle = scan->angle_min;
  for (int i = 0; i < scan->ranges.size(); i++) {
    if(angle > this->minAngle && angle < maxAngle){
      if (scan->ranges[i] < this->collDist) {
        coll = true;
        collisions.push_back(true);
      }
      collisions.push_back(false);
    }
    angle += scan->angle_increment;
  }

  

  // Find angle of collision
  int best_consec_coll = 0;
  float best_start_angle = 0;
  float best_end_angle = 0;

  int curr_consec_coll = 0;
  float curr_start_angle = 0;
  float curr_end_angle = 0;
  bool consec = false;

  // Iterate over collisions while incrementing angle
  // If collision and ! consec, consec = true, curr_start and end angle = angle, curr consec = 1
  // If collision and consec, curr end angle = angle, curr consec ++, see if curr is best
  // If !collision and consec, consec = false, 
  if(coll){
    angle = scan->angle_min;
    for (int i = 0; i < collisions.size(); i++) {
      if(collisions.at(i)){
        // If collidid and not a consec, start consec
        if(!consec){
          consec = true;
          curr_start_angle = angle;
          curr_end_angle = angle;
          curr_consec_coll = 1;
        }else{
          // Otherwise update consec and check if current consec is best
          curr_consec_coll ++;
          curr_end_angle = angle;

          if(best_consec_coll < curr_consec_coll){
            best_start_angle = curr_start_angle;
            best_end_angle = curr_end_angle;
            best_consec_coll = curr_consec_coll;
          }
        }
      }else if(consec){
        // If there isn't a collision at the next index consec = false
        if(!collisions.at((i+1) % collisions.size())){
          // Set consec to false 
          consec = false; 
        }
      }

      angle += scan->angle_increment;
    }

    // Set angle of collision to the middle of best coll
    this->collisionAngle = (best_end_angle + best_start_angle) / 2.0;
  }

  collisionFlag = coll;
  ROS_DEBUG_STREAM("No collisions.");
}

bool Sensor::returnCollisionFlag() {
  return collisionFlag;
}

float Sensor::returnCollisionAngle(){
  return collisionAngle;
}

