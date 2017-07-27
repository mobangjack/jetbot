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

ros::Publisher* pubptr;

#define UWB_T_PRECISION 0.15
#define UWB_R_PRECISION 0.03

void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& uwb_pose, const geometry_msgs::PoseWithCovarianceStampedConstPtr& amcl_pose)
{
  if (pubptr == NULL) return;

  double dx = uwb_pose->pose.pose.position.x - amcl_pose->pose.pose.position.x;
  double dy = uwb_pose->pose.pose.position.y - amcl_pose->pose.pose.position.y;
  double dt = sqrt(dx * dx + dy * dy);

  double uwb_yaw = tf::getYaw(uwb_pose->pose.pose.orientation);
  double amcl_yaw = tf::getYaw(amcl_pose->pose.pose.orientation);
  double dr = fabs(uwb_yaw - amcl_yaw);

  if (dt > UWB_T_PRECISION || dr > UWB_R_PRECISION)
  {
    geometry_msgs::PoseWithCovarianceStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map";

    if (dt > UWB_T_PRECISION)
    {
      pose_stamped.pose.pose.position.x = uwb_pose->pose.pose.position.x;
      pose_stamped.pose.pose.position.y = uwb_pose->pose.pose.position.y;
      pose_stamped.pose.pose.position.z = uwb_pose->pose.pose.position.z;
      
    }
    if (dr > UWB_R_PRECISION)
    {
      pose_stamped.pose.pose.orientation = uwb_pose->pose.pose.orientation;
    }
    else
    {
      pose_stamped.pose.pose.orientation = amcl_pose->pose.pose.orientation;
    }

    pubptr->publish(pose_stamped);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jetbot_relocator");

  int spin_rate = 10;
  
  ros::NodeHandle np("~");
  np.param<int>("spin_rate", spin_rate, 10); 

  ros::NodeHandle n;

  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> uwb_pose_sub(n, "jetbot_uwb_pose", 100);  
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> amcl_pose_sub(n, "amcl_pose", 100);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 100);
  pubptr = &pose_pub;

  message_filters::TimeSynchronizer<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> sync(uwb_pose_sub, amcl_pose_sub, 100);  
  sync.registerCallback(boost::bind(&callback, _1, _2));  
  
  ros::Rate rate(spin_rate);

  while (ros::ok())
  {
    
    ros::spinOnce();

    // TODO
    
    rate.sleep();
  }

  return 0;
}


