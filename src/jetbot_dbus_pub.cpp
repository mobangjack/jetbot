
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

#include "jetbot/DBus.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "jetbot_dbus_pub");

  int spin_rate = 20;
  
  ros::NodeHandle np("~");
  np.param<int>("spin_rate", spin_rate, 20); 
  
  ros::NodeHandle n;

  ros::Publisher dbus_pub = n.advertise<jetbot::DBus>("jetbot_dbus_pub/dbus", 100);
   
  ros::Rate rate(spin_rate);

  jetbot::DBus dbus;

  dbus.rcp.ch1 = CH_MID;
  dbus.rcp.ch2 = CH_MID;
  dbus.rcp.ch3 = CH_MID;
  dbus.rcp.ch4 = CH_MID;
  dbus.rcp.sw1 = SW_MD; 
  dbus.rcp.sw2 = SW_DN; // To obtain control privilege

  dbus.hcp.mouse_speed_x = 0;
  dbus.hcp.mouse_speed_y = 0;
  dbus.hcp.mouse_speed_z = 0;
  dbus.hcp.mouse_button_left = 0;
  dbus.hcp.mouse_button_right = 0;
  dbus.hcp.key = 0;
  dbus.hcp.res = 0;

  while (ros::ok())
  {

    ros::spinOnce();
    
    dbus_pub.publish(dbus);

    rate.sleep();
  }

  return 0;
}



