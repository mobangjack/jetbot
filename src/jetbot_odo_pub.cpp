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

#include <sstream>

#include "asp.h"
#include "uart.h"

#include "jetbot/Bot.h"

ros::Publisher* odo_pub;
  
static double px = 0;
static double py = 0;
static double pz = 0;
static ros::Time current_time, last_time;

void botCallback(const jetbot::Bot::ConstPtr& bot)
{
  
  float cvy = -bot->cbus.cv.x / BOT_MSG_VALUE_SCALE;
  float cvx = bot->cbus.cv.y / BOT_MSG_VALUE_SCALE;
  float cvz = -bot->cbus.cv.z / BOT_MSG_VALUE_SCALE;
  
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  
  last_time = current_time;
  
  double vx = cvx * cos(pz) - cvy * sin(pz);
  double vy = cvx * sin(pz) + cvy * cos(pz);
  double vz = cvz;
  
  double delta_px = vx * dt;
  double delta_py = vy * dt;
  double delta_pz = vz * dt;
  
  px += delta_px;
  py += delta_py;
  pz += delta_pz;
  
  static tf::TransformBroadcaster odom_broadcaster;
  
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pz);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = px;
  odom_trans.transform.translation.y = py;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = px;
  odom.pose.pose.position.y = py;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vz; 

  odo_pub->publish(odom);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jetbot_odo_pub");

  int uart_fd = -1;
  int spin_rate = 50;
  
  ros::NodeHandle np("~");
  np.param<int>("spin_rate", spin_rate, 50); 

  ros::NodeHandle n;

  ros::Subscriber bot_sub = n.subscribe<jetbot::Bot>("jetbot_msg_puller/bot", 100, botCallback); // Odometry listenner
  
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100); // Built-in odom

  odo_pub = &odom_pub;
  
  ros::Rate rate(spin_rate);

  while (ros::ok())
  {
    
    ros::spinOnce();

    // TODO
    
    rate.sleep();
  }

  // Done
  uart_close(uart_fd);

  return 0;
}


