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

#include <message_filters/subscriber.h>  
#include <message_filters/time_synchronizer.h>  

#include "std_msgs/String.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sstream>

#include "asp.h"

#include "jetbot/Uwb.h"

ros::Publisher* pubptr;

void uwbCallback(const jetbot::Uwb::ConstPtr& uwb)
{
  if (pubptr == NULL) return;

  static tf::TransformBroadcaster odom_broadcaster;
  
  geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(uwb->w*PI/180.0);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped trans;
  trans.header.stamp = ros::Time::now();
  trans.header.frame_id = "uwb";
  trans.child_frame_id = "base_link";

  trans.transform.translation.x = uwb->x;
  trans.transform.translation.y = uwb->y;
  trans.transform.translation.z = uwb->z;
  trans.transform.rotation = quaternion;

  geometry_msgs::PoseWithCovarianceStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = "map";
  
  pose_stamped.pose.pose.position.x = uwb->x;
  pose_stamped.pose.pose.position.y = uwb->y;
  pose_stamped.pose.pose.position.z = uwb->z;
  
  pose_stamped.pose.pose.orientation = quaternion;
  
  pubptr->publish(pose_stamped);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jetbot_uwb_pose");

  int spin_rate = 50;
  
  ros::NodeHandle np("~");
  np.param<int>("spin_rate", spin_rate, 50); 

  ros::NodeHandle n;

  ros::Subscriber uwb_sub = n.subscribe<jetbot::Uwb>("jetbot_uwb", 100, uwbCallback); // Odometry listenner
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("jetbot_uwb_pose", 100);

  pubptr = &pose_pub;
  
  ros::Rate rate(spin_rate);

  while (ros::ok())
  {
    
    ros::spinOnce();

    // TODO
    
    rate.sleep();
  }

  return 0;
}


