
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

#include "jetbot/Rcp.h"
#include "jetbot/Hcp.h"
#include "jetbot/DBus.h"

#include "jetbot/VRC.h"
#include "jetbot/VHC.h"
#include "jetbot/VDBus.h"

#include "asp.h"

ros::Publisher* vrc_pub_ptr = NULL;
ros::Publisher* vhc_pub_ptr = NULL;
ros::Publisher* vdbus_pub_ptr = NULL;

void rcpCallback(const jetbot::Rcp::ConstPtr& rcpPtr)
{
  if (vrc_pub_ptr == NULL) return;

  Rcp_t rcp;
  uint8_t buf[RCP_FRAME_LEN];
  
  Rcp_Init(&rcp);
  
  rcp.ch[0] = rcpPtr->ch1;
  rcp.ch[1] = rcpPtr->ch2;
  rcp.ch[2] = rcpPtr->ch3;
  rcp.ch[3] = rcpPtr->ch4;
  rcp.sw[0] = rcpPtr->sw1;
  rcp.sw[1] = rcpPtr->sw2;

  Rcp_Enc(&rcp, buf);

  jetbot::VRC vrc;
  for (int i = 0; i < RCP_FRAME_LEN; i++)
  {
    vrc.data.push_back(buf[i]);
  }
  
  vrc_pub_ptr->publish(vrc);
}

void hcpCallback(const jetbot::Hcp::ConstPtr& hcpPtr)
{
  if (vhc_pub_ptr == NULL) return;

  Hcp_t hcp;
  uint8_t buf[HCP_FRAME_LEN];

  Hcp_Init(&hcp);

  hcp.mouse.x = hcpPtr->mouse_speed_x;
  hcp.mouse.y = hcpPtr->mouse_speed_y;
  hcp.mouse.z = hcpPtr->mouse_speed_z;
  hcp.mouse.b[0] = hcpPtr->mouse_button_left;
  hcp.mouse.b[1] = hcpPtr->mouse_button_right;
  hcp.key.val = hcpPtr->key;
  hcp.res.val = hcpPtr->res;

  Hcp_Enc(&hcp, buf);

  jetbot::VHC vhc;
  for (int i = 0; i < HCP_FRAME_LEN; i++)
  {
    vhc.data.push_back(buf[i]);
  }
  
  vhc_pub_ptr->publish(vhc);
}

void dbusCallback(const jetbot::DBus::ConstPtr& dbusPtr)
{
  if (vdbus_pub_ptr == NULL) return;

  DBus_t dbus;
  uint8_t buf[DBUS_FRAME_LEN];
  
  DBus_Init(&dbus);

  dbus.rcp.ch[0] = dbusPtr->rcp.ch1;
  dbus.rcp.ch[1] = dbusPtr->rcp.ch2;
  dbus.rcp.ch[2] = dbusPtr->rcp.ch3;
  dbus.rcp.ch[3] = dbusPtr->rcp.ch4;
  dbus.rcp.sw[0] = dbusPtr->rcp.sw1;
  dbus.rcp.sw[1] = dbusPtr->rcp.sw2;

  dbus.hcp.mouse.x = dbusPtr->hcp.mouse_speed_x;
  dbus.hcp.mouse.y = dbusPtr->hcp.mouse_speed_y;
  dbus.hcp.mouse.z = dbusPtr->hcp.mouse_speed_z;
  dbus.hcp.mouse.b[0] = dbusPtr->hcp.mouse_button_left;
  dbus.hcp.mouse.b[1] = dbusPtr->hcp.mouse_button_right;
  dbus.hcp.key.val = dbusPtr->hcp.key;
  dbus.hcp.res.val = dbusPtr->hcp.res;

  DBus_Enc(&dbus, buf);

  jetbot::VDBus vdbus;
  for (int i = 0; i < DBUS_FRAME_LEN; i++)
  {
    vdbus.data.push_back(buf[i]);
  }
  
  vdbus_pub_ptr->publish(vdbus);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "jetbot_virtual_rc");

  int spin_rate = 50;
  
  ros::NodeHandle np("~");
  np.param<int>("spin_rate", spin_rate, 50); 
  
  ros::NodeHandle n;

  ros::Subscriber rcp_sub = n.subscribe<jetbot::Rcp>("jetbot_virtual_rc/rcp", 100, rcpCallback); 
  ros::Subscriber hcp_sub = n.subscribe<jetbot::Hcp>("jetbot_virtual_rc/hcp", 100, hcpCallback); 
  ros::Subscriber dbus_sub = n.subscribe<jetbot::DBus>("jetbot_virtual_rc/dbus", 100, dbusCallback); 
  
  ros::Publisher vrc_pub = n.advertise<jetbot::VDBus>("jetbot_msg_pusher/vrc", 100);
  ros::Publisher vhc_pub = n.advertise<jetbot::VDBus>("jetbot_msg_pusher/vhc", 100);
  ros::Publisher vdbus_pub = n.advertise<jetbot::VDBus>("jetbot_msg_pusher/vdbus", 100);

  vrc_pub_ptr = &vrc_pub;
  vhc_pub_ptr = &vhc_pub;
  vdbus_pub_ptr = &vdbus_pub;
   
  ros::Rate rate(spin_rate);

  while (ros::ok())
  {

    ros::spinOnce();
  
    rate.sleep();
  }

  return 0;
}



