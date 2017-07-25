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

#include <sstream>

#include "asp.h"
#include "uart.h"

#include "jetbot/Bot.h"
#include "jetbot/Uwb.h"
#include "jetbot/VDBus.h"
#include "jetbot/ZGyro.h"

#define BUF_LEN 128

void publishBotMsg(ros::Publisher *pub, const BotMsg_t* botMsg)
{
  jetbot::Bot bot;
  
  bot.frame_id = botMsg->frame_id;
  bot.cbus.fs = botMsg->cbus.fs;
  bot.cbus.cp.x = botMsg->cbus.cp.x;
  bot.cbus.cp.y = botMsg->cbus.cp.y;
  bot.cbus.cp.z = botMsg->cbus.cp.z;
  bot.cbus.cv.x = botMsg->cbus.cv.x;
  bot.cbus.cv.y = botMsg->cbus.cv.y;
  bot.cbus.cv.z = botMsg->cbus.cv.z;
  bot.cbus.gp.p = botMsg->cbus.gp.p;
  bot.cbus.gp.t = botMsg->cbus.gp.t;
  //bot.cbus.gp.z = botMsg->cbus.gp.z;
  bot.cbus.gv.p = botMsg->cbus.gv.p;
  bot.cbus.gv.t = botMsg->cbus.gv.t;
  //bot.cbus.gv.z = botMsg->cbus.gv.z;
  
  pub->publish(bot);
}

void publishUwbMsg(ros::Publisher *pub, const UwbMsg_t* uwbMsg)
{
  jetbot::Uwb uwb;
  
  uwb.frame_id = uwbMsg->frame_id;
  uwb.flag = uwbMsg->flag;
  uwb.x = uwbMsg->x;
  uwb.y = uwbMsg->y;
  uwb.z = uwbMsg->z;
  uwb.w = uwbMsg->w;
 
  pub->publish(uwb);
}

void publishZGyroMsg(ros::Publisher *pub, const ZGyroMsg_t* zgyroMsg)
{
  jetbot::ZGyro zgyro;
  
  zgyro.frame_id = zgyroMsg->frame_id;
  zgyro.angle = zgyroMsg->angle;
  zgyro.rate = zgyroMsg->rate;
  
  pub->publish(zgyro);
}

void publishVDBusMsg(ros::Publisher *pub, const VDBusMsg_t* vdbusMsg)
{
  jetbot::VDBus vdbus;
  
  vdbus.frame_id = vdbusMsg->frame_id;
  
  for (int i = 0; i < DBUS_FRAME_LEN; i++) {
    vdbus.data.push_back(vdbusMsg->data[i]);
  }
  
  pub->publish(vdbus);
}

int main(int argc, char **argv)
{
  int uart_fd = -1;
  
  FIFO_t rx_fifo;
  uint8_t rx_buf[2][BUF_LEN];

  FIFO_Init(&rx_fifo, rx_buf[0], BUF_LEN);

  UwbMsg_t uwbMsg;
  BotMsg_t botMsg;
  ZGyroMsg_t zgyroMsg;
  VDBusMsg_t vdbusMsg;
  
  ros::init(argc, argv, "jetbot_msg_puller");

  std::string serial_port;
  int serial_baudrate = 115200;
  int spin_rate = 100;
  
  ros::NodeHandle np("~");
  np.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2"); 
  np.param<int>("serial_baudrate", serial_baudrate, 115200);
  np.param<int>("spin_rate", spin_rate, 100); 

  int ret = uart_open(&uart_fd, serial_port.c_str(), serial_baudrate, UART_OFLAG_RD);
  if (ret < 0) {
    fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , serial_port.c_str());
    return -1;
  }
  
  ros::NodeHandle n;

  ros::Publisher bot_pub = n.advertise<jetbot::Bot>("jetbot_msg_puller/bot", 100); // Bot odo message feedback
  ros::Publisher uwb_pub = n.advertise<jetbot::Uwb>("jetbot_msg_puller/uwb", 100); // UWB Locater
  ros::Publisher zgyro_pub = n.advertise<jetbot::ZGyro>("jetbot_msg_puller/zgyro", 100); // ZGyro feedback
  ros::Publisher vdbus_pub = n.advertise<jetbot::VDBus>("jetbot_msg_puller/vdbus", 100); // VDBus feedback
  
  ros::Rate rate(spin_rate);

  while (ros::ok())
  {
	
    ros::spinOnce();

    // Get fifo free space
    int len = FIFO_GetFree(&rx_fifo);
    
    // If fifo free space insufficient, pop one element out
    if (len < 1) {
      uint8_t b;
      len = FIFO_Pop(&rx_fifo, &b, 1);
    }
    
    // Read input stream according to the fifo free space left
    len = uart_read(uart_fd, rx_buf[1], len);

    // Check if there is any new stream
    if (len > 0) {
      // Push new stream into fifo
      FIFO_Push(&rx_fifo, rx_buf[1], len);
    }

    // Check if any message received
    
    if (Msg_Pop(&rx_fifo, rx_buf[1], &MSG_HEAD_OF(BOT), &botMsg)) {
      publishBotMsg(&bot_pub, &botMsg);
    }

    if (Msg_Pop(&rx_fifo, rx_buf[1], &MSG_HEAD_OF(UWB), &uwbMsg)) {
      publishUwbMsg(&uwb_pub, &uwbMsg);
    }
    
    if (Msg_Pop(&rx_fifo, rx_buf[1], &MSG_HEAD_OF(ZGYRO), &zgyroMsg)) {
      publishZGyroMsg(&zgyro_pub, &zgyroMsg);
    }

    if (Msg_Pop(&rx_fifo, rx_buf[1], &MSG_HEAD_OF(VDBUS), &vdbusMsg)) {
      publishVDBusMsg(&vdbus_pub, &vdbusMsg);
    }

    rate.sleep();
  }

  // Done
  uart_close(uart_fd);

  return 0;
}


