
/**
 * Copyright (c) 2016, Jack Mo (mobangjack@foxmail.com).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

#include "asp.h"

ros::Time lastPubTime;

ros::Publisher* pubptr = NULL;
geometry_msgs::Twist twist;

void velCallback(const geometry_msgs::Twist::ConstPtr& twistPtr)
{
  if (pubptr == NULL) return;

  twist.linear.x = -twistPtr->linear.y;
  twist.linear.y = twistPtr->linear.x;
  twist.angular.z = -twistPtr->angular.z;
  
  //pubptr->publish(twist);
  lastPubTime = ros::Time::now();
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "jetbot_vel_repub");

  int spin_rate = 50;
  int desire_rate = 20;
  
  
  ros::NodeHandle np("~");
  np.param<int>("spin_rate", spin_rate, 50);
  np.param<int>("desire_rate", spin_rate, 20);

  if (desire_rate < 1) desire_rate = 1;

  double desire_period = 1.0 / desire_rate;
  
  ros::NodeHandle n;

  ros::Subscriber vel_sub = n.subscribe<geometry_msgs::Twist>("jetbot_cmd_vel/cmd_vel", 100, velCallback);
  
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("jetbot_vel_repub/cmd_vel", 100);


  pubptr = &vel_pub;
   
  ros::Rate rate(spin_rate);

  double timeLap = 0;

  while (ros::ok())
  {

    ros::spinOnce();

    timeLap = (ros::Time::now() - lastPubTime).toSec();
    if (timeLap > desire_period)
    {
      vel_pub.publish(twist);
    }

    rate.sleep();
  }

  return 0;
}



