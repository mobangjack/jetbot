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

#include <iostream>
#include <sstream>

#include "asp.h"
#include "uart.h"

#include "jetbot/Bot.h"
#include "jetbot/VRC.h"
#include "jetbot/VHC.h"
#include "jetbot/VDBus.h"

#define BUF_LEN 256

int uart_fd = -1;
  
uint8_t tx_buf[BUF_LEN];

BotMsg_t botMsg;
VRCMsg_t vrcMsg;
VHCMsg_t vhcMsg;
VDBusMsg_t vdbusMsg;
  
void botCallback(const jetbot::Bot::ConstPtr& bot)
{
  botMsg.frame_id++;
  botMsg.cbus.fs = bot->cbus.fs;
  botMsg.cbus.cp.x = bot->cbus.cp.x;
  botMsg.cbus.cp.y = bot->cbus.cp.y;
  botMsg.cbus.cp.z = bot->cbus.cp.z;
  botMsg.cbus.cv.x = bot->cbus.cv.x;
  botMsg.cbus.cv.y = bot->cbus.cv.y;
  botMsg.cbus.cv.z = bot->cbus.cv.z;
  botMsg.cbus.gp.p = bot->cbus.gp.p;
  botMsg.cbus.gp.t = bot->cbus.gp.t;
  //botMsg.cbus.gp.z = bot->cbus.gp.z;
  botMsg.cbus.gv.p = bot->cbus.gv.p;
  botMsg.cbus.gv.t = bot->cbus.gv.t;
  //botMsg.cbus.gv.z = bot->cbus.gv.z;

  //std::cout << "Bot is called" << std::endl;
  
  uint32_t len = Msg_Pack(tx_buf, &MSG_HEAD_OF(BOT), &botMsg);
  uart_write(uart_fd, tx_buf, len);
  
}

void vrcCallback(const jetbot::VRC::ConstPtr& vrc)
{
  vrcMsg.frame_id++;

  for (int i = 0; i < vrc->data.size(); i++)
  {
    vrcMsg.data[i] = vrc->data[i];
  }
  
  uint32_t len = Msg_Pack(tx_buf, &MSG_HEAD_OF(VRC), &vrcMsg);
  uart_write(uart_fd, tx_buf, len);
  
}

void vhcCallback(const jetbot::VHC::ConstPtr& vhc)
{
  vhcMsg.frame_id++;

  for (int i = 0; i < vhc->data.size(); i++)
  {
    vhcMsg.data[i] = vhc->data[i];
  }
  
  uint32_t len = Msg_Pack(tx_buf, &MSG_HEAD_OF(VHC), &vhcMsg);
  uart_write(uart_fd, tx_buf, len);
  
}

void vdbusCallback(const jetbot::VDBus::ConstPtr& vdbus)
{
  vdbusMsg.frame_id++;

  //std::cout << "vdbus received: ";
  for (int i = 0; i < vdbus->data.size(); i++)
  {
    vdbusMsg.data[i] = vdbus->data[i];
    //std::cout << (int)vdbus->data[i] << " ";
  }
  //std::cout << std::endl;

  DBus_t dbus;
  DBus_Dec(&dbus, vdbusMsg.data);
  /*
  std::cout << "dbus decode------ " << \
          "id:" << vdbusMsg.frame_id << "," << \
          "ch1:" << dbus.rcp.ch[0] << "," << \
          "ch2:" << dbus.rcp.ch[1] << "," << \
          "ch3:" << dbus.rcp.ch[2] << "," << \
          "ch4:" << dbus.rcp.ch[3] << "," << \
          "sw1:" << (int)dbus.rcp.sw[0] << "," << \
          "sw2:" << (int)dbus.rcp.sw[1] << "," << \
          "msx:" << (int)dbus.hcp.mouse.x << "," << \
          "msy:" << (int)dbus.hcp.mouse.y << "," << \
          "msz:" << (int)dbus.hcp.mouse.z << "," << \
          "key:" << dbus.hcp.key.val << std::endl;
          */
  uint32_t len = Msg_Pack(tx_buf, &MSG_HEAD_OF(VDBUS), &vdbusMsg);
  uart_write(uart_fd, tx_buf, len);
  
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "jetbot_msg_pusher");

  std::string serial_port;
  int serial_baudrate = 115200;
  int spin_rate = 50;
  
  ros::NodeHandle np("~");
  np.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2"); 
  np.param<int>("serial_baudrate", serial_baudrate, 115200);
  np.param<int>("spin_rate", spin_rate, 50); 
  
  int ret = uart_open(&uart_fd, serial_port.c_str(), serial_baudrate, UART_OFLAG_WR);
  if (ret < 0) {
    fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , serial_port.c_str());
    return -1;
  }

  ros::NodeHandle n;

  ros::Subscriber bot_sub = n.subscribe<jetbot::Bot>("jetbot_msg_pusher/bot", 100, botCallback); // Command listenner
  ros::Subscriber vrc_sub = n.subscribe<jetbot::VRC>("jetbot_msg_pusher/vrc", 100, vrcCallback);
  ros::Subscriber vhc_sub = n.subscribe<jetbot::VHC>("jetbot_msg_pusher/vhc", 100, vhcCallback);
  ros::Subscriber vdbus_sub = n.subscribe<jetbot::VDBus>("jetbot_msg_pusher/vdbus", 100, vdbusCallback);

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


