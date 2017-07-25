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

#ifndef __JUD_H__
#define __JUD_H__

#include "crc8.h"
#include "crc16.h"
#include <string.h>

#define JUD_FRAME_BUF_LEN 100

#define JUD_SOF 0xA5
#define JUD_HEADER_LEN sizeof(JudFrameHeader_t)
#define JUD_CMD_ID_LEN 2
#define JUD_CRC_LEN 2
#define JUD_DATA_OFFSET (JUD_HEADER_LEN + JUD_CMD_ID_LEN)
#define JUD_EXT_LEN (JUD_HEADER_LEN + JUD_CMD_ID_LEN + JUD_CRC_LEN)
#define JUD_GET_FRAME_LEN(DATA_LENGTH) ((DATA_LENGTH) + JUD_EXT_LEN)

//crc8 generator polynomial:G(x) = x8+x5+x4+1
#define  JUD_CRC8_INIT 0xff
#define  JUD_CRC16_INIT 0xffff

#pragma pack(1)

typedef uint16_t JudCmdId_t;

#define JUD_CMD_ID_INVALID         ((uint16_t)0x0000)
#define JUD_CMD_ID_GAME_INFO       ((uint16_t)0x0001)
#define JUD_CMD_ID_RT_BLOOD_CHANGE ((uint16_t)0x0002)
#define JUD_CMD_ID_RT_SHOOT_DATA   ((uint16_t)0x0003)

typedef struct
{
    uint8_t sof;
    uint16_t dataLength;
    uint8_t seq;
    uint8_t crc8;
} JudFrameHeader_t;

typedef struct
{
	uint8_t flag; // 0:invalid 1:valid
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t w;
} JudGps_t;

/** 
  * @brief  Game information structures definition(0x0001)
  *         this package send frequency is 50Hz
  */
typedef struct
{
    uint32_t remainTime; // 3min down-counter, unit: s
    uint16_t remainLifeValue; // blood value
    float    RTChassisVotage; // unit: V
    float    RTChassisCurrent; // unit: A
    JudGps_t gps;
    float    remainPower; // unit: J.//max = 60J
} JudGameInfo_t;

/** 
  * @brief  Real time blood change status (0x0002)
  */
typedef struct
{
    uint8_t weakId : 4;
    // 0-3bits: ID for armor hurt:
    // 0x00: 0 front
    // 0x01： 1 left
    //  0x02： 2 back
    // 0x03： 3 right
    // 0x04: 4 top1
    // 0x05: 5 top2
    uint8_t way : 4;
    // 4-7bits: blood change type
    // 0x0: armor hurt
    // 0x1: bullet speed out of limit
    // 0x2: shoot frequency out of limit
    // 0x3: power out of limit
    // 0x4: module offline
    // 0x6: breaking rules
    // 0xa: gain  blood-charge card
    // 0xb: engineer auto recovery.
    uint16_t value; // blood change value
} JudRTBloodChange_t;

/** 
  * @brief  Real time shooting speed info (0x0003)
  */
typedef struct
{
    float RTBulletShootSpeed; // m/s
    float RTBulletShootFreq; // bullet/sec
    float RTGolfShootSpeed; // m/s
    float RTGolfShootFreq; // bullet/sec
} JudRTShootData_t;

#define JUD_CUSTOM_DATA_SIZE 3
typedef struct
{
    float data[JUD_CUSTOM_DATA_SIZE];
} JudCustomData_t;

typedef union {
    uint8_t U8[4];
    float F32;
} JudUploadPC_u;

#pragma pack()

JudFrameHeader_t* Jud_GetFrameHeader(const void* buf);
void* Jud_GetData(const void* buf);
JudCmdId_t Jud_GetCmdId(const void* buf);

#endif
