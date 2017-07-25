
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
#include "uart.h"

#include "jetbot/Bot.h"

ros::Publisher* pubptr = NULL;
jetbot::Bot bot;

void velCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
  if (pubptr == NULL) return;

  bot.frame_id++;

  bot.cbus.cv.x = -twist->linear.y * 1000;
  bot.cbus.cv.y = twist->linear.x * 1000;
  bot.cbus.cv.z = -twist->angular.z * 1000;
  
  pubptr->publish(bot);

}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "jetbot_cmd_vel");

  int spin_rate = 100;
  
  ros::NodeHandle np("~");
  np.param<int>("spin_rate", spin_rate, 30); 
  
  ros::NodeHandle n;

  ros::Subscriber vel_sub = n.subscribe<geometry_msgs::Twist>("jetbot_cmd_vel/cmd_vel", 100, velCallback); // Odometry feedback listenner
  
  ros::Publisher bot_pub = n.advertise<jetbot::Bot>("jetbot_msg_pusher/bot", 100); // Command advertiser

  pubptr = &bot_pub;
   
  ros::Rate rate(spin_rate);

  while (ros::ok())
  {

    ros::spinOnce();
  
    rate.sleep();
  }

  return 0;
}



