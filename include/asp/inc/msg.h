/**
 * Copyright (c) 2011-2016, Jack Mo (mobangjack@foxmail.com).
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
 
#ifndef __MSG_H__
#define __MSG_H__

/********************************************************/
/*                 KylinBot Msg Type                    */
/*     Basic frame structure:                           */
/*       ________________________________________       */
/*      |id:8|length:8|token:16|data~|checksum:16|      */
/*      |______________________|_____|___________|      */
/*      |         head         |body~|    crc    |      */
/*      |________________________________________|      */
/*                         Msg                          */
/********************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "cbus.h"
#include "dbus.h"
#include "fifo.h"
#include "calib.h"
#include "crc16.h"
	
#define MSG_HEAD_LEN sizeof(MsgHead_t)
#define MSG_CRC_LEN 2
#define MSG_BASE_LEN (MSG_HEAD_LEN + MSG_CRC_LEN)

#define MSG_GET_LEN(DATA_LENGTH) ((DATA_LENGTH) + MSG_BASE_LEN)
#define MSG_HEAD_OF(MSG) msg_head[MSG_TYPE_IDX_##MSG]
#define MSG_LEN_OF(MSG) MSG_GET_LEN(msg_head[MSG_TYPE_IDX_##MSG].attr.dataLength)

#pragma pack(1)

/* Message head union typedef */
typedef union MsgHead_t
{
	uint32_t value; // Message head value in 32bit
	struct
	{
		uint8_t id : 8; // Message ID
		uint8_t dataLength : 8; // Message data length (unit: byte)
		uint16_t token : 16; // Message CRC token
	}attr; // Message head attributes
}MsgHead_t; // Message head union typedef

#define BOT_MSG_VALUE_SCALE CBUS_VALUE_SCALE
typedef struct
{
	uint32_t frame_id;
	CBus_t cbus; // Control Bus
}BotMsg_t;

#define IMU_MSG_VALUE_SCALE 1.0f
typedef struct
{
	uint32_t frame_id;
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
}ImuMsg_t;

#define MAG_MSG_VALUE_SCALE 1.0f
typedef struct
{
	uint32_t frame_id;
	int16_t mx;
	int16_t my;
	int16_t mz;
}MagMsg_t;

typedef struct
{
	uint32_t frame_id;
	uint8_t  flag; //0:invalid 1:valid
	uint32_t x;
	uint32_t y;
	uint32_t z;
	uint32_t w;
}UwbMsg_t;

#define PTZ_MSG_VALUE_SCALE 1e3f
typedef struct
{
	uint32_t frame_id;
	int16_t p; // pan
	int16_t t; // tilt
	int16_t z; // zoom
}PTZMsg_t;

typedef struct
{
	uint32_t frame_id;
	uint8_t data[RCP_FRAME_LEN];
}VRCMsg_t;

typedef struct
{
	uint32_t frame_id;
	uint8_t data[HCP_FRAME_LEN];
}VHCMsg_t;

#define AHRS_MSG_VALUE_SCALE 1.0f
typedef struct
{
	uint32_t frame_id;
	float q[4];
}AHRSMsg_t;

typedef struct
{
	uint32_t frame_id;
	uint8_t data[DBUS_FRAME_LEN];
}VDBusMsg_t;

#define ZGYRO_ANGLE_RECIP 1e-2f // To scale zgyro angle to deg
#define ZGYRO_RATE_RECIP 1e-5f // To scale zgyro rate to deg/s
typedef struct
{
	uint32_t frame_id;
	int32_t angle; // = (deg*100)
	int16_t rate; // = delta(angle)/1ms
}ZGyroMsg_t;

typedef enum
{
	MOTOR1_ID,
	MOTOR2_ID,
	MOTOR3_ID,
	MOTOR4_ID,
	MOTOR5_ID,
	MOTOR6_ID,
}MotorId_e;
#define MOTOR_ECD_ANGLE_MAX 8191
typedef struct
{
	uint8_t id; // 0~5
	uint32_t frame_id;
	uint16_t ecd_angle; // Encoder angle, range from 0~8191
	int32_t round;
	int32_t angle; // Continuous angle, infinite
	int16_t rate; // Rate in ecd_diff/1ms
}MotorMsg_t;

typedef struct
{
	uint32_t frame_id;
	uint32_t wdg; // Watchdog
	uint32_t ini; // Initialization status
}StatuMsg_t;

typedef struct
{
	uint32_t frame_id;
	uint32_t msg_type;
}SubscMsg_t;

#define CALIB_FLAG_BIT_IMU (1u<<0)
#define CALIB_FLAG_BIT_MAG (1u<<1)
#define CALIB_FLAG_BIT_POS (1u<<2)
typedef struct
{
	uint32_t frame_id;
	uint32_t auto_cali_flag; // Auto calibration control bits
}CalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	PIDCalib_t data;
}PIDCalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	IMUCalib_t data;
}IMUCalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	MagCalib_t data;
}MagCalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	VelCalib_t data;
}VelCalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	PosCalib_t data;
}PosCalibMsg_t;

typedef struct
{
	uint32_t frame_id;
	MecCalib_t data;
}MecCalibMsg_t;

#define WRAP_U8(V) ((uint8_t)V)
#define WRAP_U16(V) ((uint16_t)V)
#define WRAP_U32(V) ((uint32_t)V)

typedef enum
{
	MSG_ID_BOT,
	MSG_ID_IMU,
	MSG_ID_MAG,
	MSG_ID_UWB,
	MSG_ID_PTZ,
	MSG_ID_VRC,
	MSG_ID_VHC,
	MSG_ID_AHRS,
	MSG_ID_VDBUS,
	MSG_ID_ZGYRO,
	MSG_ID_MOTOR,
	MSG_ID_STATU,
	MSG_ID_SUBSC,
	MSG_ID_CALIB,
	MSG_ID_PID_CALIB,
	MSG_ID_IMU_CALIB,
	MSG_ID_MAG_CALIB,
	MSG_ID_VEL_CALIB,
	MSG_ID_POS_CALIB,
	MSG_ID_MEC_CALIB,
}MsgId_e;

#define MSG_LEN_BOT sizeof(BotMsg_t)
#define MSG_LEN_IMU sizeof(ImuMsg_t)
#define MSG_LEN_MAG sizeof(MagMsg_t)
#define MSG_LEN_UWB sizeof(UwbMsg_t)
#define MSG_LEN_PTZ sizeof(PTZMsg_t)
#define MSG_LEN_VRC sizeof(VRCMsg_t)
#define MSG_LEN_VHC sizeof(VHCMsg_t)
#define MSG_LEN_AHRS sizeof(AHRSMsg_t)
#define MSG_LEN_VDBUS sizeof(VDBusMsg_t)
#define MSG_LEN_ZGYRO sizeof(ZGyroMsg_t)
#define MSG_LEN_MOTOR sizeof(MotorMsg_t)
#define MSG_LEN_STATU sizeof(StatuMsg_t)
#define MSG_LEN_SUBSC sizeof(SubscMsg_t)
#define MSG_LEN_CALIB sizeof(CalibMsg_t)
#define MSG_LEN_PID_CALIB sizeof(PIDCalibMsg_t)
#define MSG_LEN_IMU_CALIB sizeof(IMUCalibMsg_t)
#define MSG_LEN_MAG_CALIB sizeof(MagCalibMsg_t)
#define MSG_LEN_VEL_CALIB sizeof(VelCalibMsg_t)
#define MSG_LEN_POS_CALIB sizeof(PosCalibMsg_t)
#define MSG_LEN_MEC_CALIB sizeof(MecCalibMsg_t)

typedef enum
{
	MSG_TOKEN_BOT = 0x1234,
	MSG_TOKEN_IMU,
	MSG_TOKEN_MAG,
	MSG_TOKEN_UWB,
	MSG_TOKEN_PTZ,
	MSG_TOKEN_VRC,
	MSG_TOKEN_VHC,
	MSG_TOKEN_AHRS,
	MSG_TOKEN_VDBUS,
	MSG_TOKEN_ZGYRO,
	MSG_TOKEN_MOTOR,
	MSG_TOKEN_STATU,
	MSG_TOKEN_SUBSC,
	MSG_TOKEN_CALIB,
	MSG_TOKEN_PID_CALIB,
	MSG_TOKEN_IMU_CALIB,
	MSG_TOKEN_MAG_CALIB,
	MSG_TOKEN_VEL_CALIB,
	MSG_TOKEN_POS_CALIB,
	MSG_TOKEN_MEC_CALIB,
}MsgToken_e;

#define MSG_HEAD_VALUE(ID,LEN,TOKEN) ((WRAP_U32(TOKEN)<<16) | (WRAP_U32(LEN)<<8) | WRAP_U32(ID))
#define MSG_HEAD_VALUE_OF(NAME) MSG_HEAD_VALUE(MSG_ID_##NAME,MSG_LEN_##NAME,MSG_TOKEN_##NAME)

#define MSG_HEAD_VALUE_BOT MSG_HEAD_VALUE_OF(BOT)
#define MSG_HEAD_VALUE_IMU MSG_HEAD_VALUE_OF(IMU)
#define MSG_HEAD_VALUE_MAG MSG_HEAD_VALUE_OF(MAG)
#define MSG_HEAD_VALUE_UWB MSG_HEAD_VALUE_OF(UWB)
#define MSG_HEAD_VALUE_PTZ MSG_HEAD_VALUE_OF(PTZ)
#define MSG_HEAD_VALUE_VRC MSG_HEAD_VALUE_OF(VRC)
#define MSG_HEAD_VALUE_VHC MSG_HEAD_VALUE_OF(VHC)
#define MSG_HEAD_VALUE_AHRS MSG_HEAD_VALUE_OF(AHRS)
#define MSG_HEAD_VALUE_VDBUS MSG_HEAD_VALUE_OF(VDBUS)
#define MSG_HEAD_VALUE_ZGYRO MSG_HEAD_VALUE_OF(ZGYRO)
#define MSG_HEAD_VALUE_MOTOR MSG_HEAD_VALUE_OF(MOTOR)
#define MSG_HEAD_VALUE_STATU MSG_HEAD_VALUE_OF(STATU)
#define MSG_HEAD_VALUE_SUBSC MSG_HEAD_VALUE_OF(SUBSC)
#define MSG_HEAD_VALUE_CALIB MSG_HEAD_VALUE_OF(CALIB)
#define MSG_HEAD_VALUE_PID_CALIB MSG_HEAD_VALUE_OF(PID_CALIB)
#define MSG_HEAD_VALUE_IMU_CALIB MSG_HEAD_VALUE_OF(IMU_CALIB)
#define MSG_HEAD_VALUE_MAG_CALIB MSG_HEAD_VALUE_OF(MAG_CALIB)
#define MSG_HEAD_VALUE_VEL_CALIB MSG_HEAD_VALUE_OF(VEL_CALIB)
#define MSG_HEAD_VALUE_POS_CALIB MSG_HEAD_VALUE_OF(POS_CALIB)
#define MSG_HEAD_VALUE_MEC_CALIB MSG_HEAD_VALUE_OF(MEC_CALIB)

#define MSG_HEAD_BOT { MSG_HEAD_VALUE_BOT }
#define MSG_HEAD_IMU { MSG_HEAD_VALUE_VRC }
#define MSG_HEAD_MAG { MSG_HEAD_VALUE_MAG }
#define MSG_HEAD_UWB { MSG_HEAD_VALUE_UWB }
#define MSG_HEAD_PTZ { MSG_HEAD_VALUE_PTZ }
#define MSG_HEAD_VRC { MSG_HEAD_VALUE_VRC }
#define MSG_HEAD_VHC { MSG_HEAD_VALUE_VHC }
#define MSG_HEAD_AHRS { MSG_HEAD_VALUE_AHRS }
#define MSG_HEAD_VDBUS { MSG_HEAD_VALUE_VDBUS }
#define MSG_HEAD_ZGYRO { MSG_HEAD_VALUE_ZGYRO }
#define MSG_HEAD_MOTOR { MSG_HEAD_VALUE_MOTOR }
#define MSG_HEAD_STATU { MSG_HEAD_VALUE_STATU }
#define MSG_HEAD_SUBSC { MSG_HEAD_VALUE_SUBSC }
#define MSG_HEAD_CALIB { MSG_HEAD_VALUE_CALIB }
#define MSG_HEAD_PID_CALIB { MSG_HEAD_VALUE_PID_CALIB }
#define MSG_HEAD_IMU_CALIB { MSG_HEAD_VALUE_IMU_CALIB }
#define MSG_HEAD_MAG_CALIB { MSG_HEAD_VALUE_MAG_CALIB }
#define MSG_HEAD_POS_CALIB { MSG_HEAD_VALUE_POS_CALIB }
#define MSG_HEAD_VEL_CALIB { MSG_HEAD_VALUE_VEL_CALIB }
#define MSG_HEAD_MEC_CALIB { MSG_HEAD_VALUE_MEC_CALIB }

#define MSG_HEAD_ARRAY \
{ \
	MSG_HEAD_BOT, \
	MSG_HEAD_IMU, \
	MSG_HEAD_MAG, \
	MSG_HEAD_UWB, \
	MSG_HEAD_PTZ, \
	MSG_HEAD_VRC, \
	MSG_HEAD_VHC, \
	MSG_HEAD_AHRS, \
	MSG_HEAD_VDBUS, \
	MSG_HEAD_ZGYRO, \
	MSG_HEAD_MOTOR, \
	MSG_HEAD_STATU, \
	MSG_HEAD_SUBSC, \
	MSG_HEAD_CALIB, \
	MSG_HEAD_PID_CALIB, \
	MSG_HEAD_IMU_CALIB, \
	MSG_HEAD_MAG_CALIB, \
	MSG_HEAD_VEL_CALIB, \
	MSG_HEAD_POS_CALIB, \
	MSG_HEAD_MEC_CALIB, \
}

#define MSG_TYPE_NUM 20

typedef enum
{
	MSG_TYPE_IDX_BOT,
	MSG_TYPE_IDX_IMU,
	MSG_TYPE_IDX_MAG,
	MSG_TYPE_IDX_UWB,
	MSG_TYPE_IDX_PTZ,
	MSG_TYPE_IDX_VRC,
	MSG_TYPE_IDX_VHC,
	MSG_TYPE_IDX_AHRS,
	MSG_TYPE_IDX_VDBUS,
	MSG_TYPE_IDX_ZGYRO,
	MSG_TYPE_IDX_MOTOR,
	MSG_TYPE_IDX_STATU,
	MSG_TYPE_IDX_SUBSC,
	MSG_TYPE_IDX_CALIB,
	MSG_TYPE_IDX_PID_CALIB,
	MSG_TYPE_IDX_IMU_CALIB,
	MSG_TYPE_IDX_MAG_CALIB,
	MSG_TYPE_IDX_VEL_CALIB,
	MSG_TYPE_IDX_MEC_CALIB,
	MSG_TYPE_IDX_POS_CALIB,
}MsgTypeIdx_e;

typedef enum
{
	MSG_TYPE_BOT = 1u << MSG_TYPE_IDX_BOT,
	MSG_TYPE_IMU = 1u << MSG_TYPE_IDX_IMU,
	MSG_TYPE_MAG = 1u << MSG_TYPE_IDX_MAG,
	MSG_TYPE_UWB = 1u << MSG_TYPE_IDX_UWB,
	MSG_TYPE_PTZ = 1u << MSG_TYPE_IDX_PTZ,
	MSG_TYPE_VRC = 1u << MSG_TYPE_IDX_VRC,
	MSG_TYPE_VHC = 1u << MSG_TYPE_IDX_VHC,
	MSG_TYPE_AHRS = 1u << MSG_TYPE_IDX_AHRS,
	MSG_TYPE_VDBUS = 1u << MSG_TYPE_IDX_VDBUS,
	MSG_TYPE_ZGYRO = 1u << MSG_TYPE_IDX_ZGYRO,
	MSG_TYPE_MOTOR = 1u << MSG_TYPE_IDX_MOTOR,
	MSG_TYPE_STATU = 1u << MSG_TYPE_IDX_STATU,
	MSG_TYPE_SUBSC = 1u << MSG_TYPE_IDX_SUBSC,
	MSG_TYPE_CALIB = 1u << MSG_TYPE_IDX_CALIB,
	MSG_TYPE_PID_CALIB = 1u << MSG_TYPE_IDX_PID_CALIB,
	MSG_TYPE_IMU_CALIB = 1u << MSG_TYPE_IDX_IMU_CALIB,
	MSG_TYPE_MAG_CALIB = 1u << MSG_TYPE_IDX_MAG_CALIB,
	MSG_TYPE_VEL_CALIB = 1u << MSG_TYPE_IDX_VEL_CALIB,
	MSG_TYPE_POS_CALIB = 1u << MSG_TYPE_IDX_POS_CALIB,
	MSG_TYPE_MEC_CALIB = 1u << MSG_TYPE_IDX_MEC_CALIB,
}MsgType_e;

#pragma pack()

void* Msg_GetData(const void* buf, const void* head);
uint32_t Msg_Pack(void* buf, const void* head, const void* body);

/**
 * @brief: Push a single message to message fifo.
 * @param fifo Message fifo
 * @param buf Message buffer
 * @param head Message head
 * @param body Message body
 * @return Message length (num of bytes)
 */
uint32_t Msg_Push(FIFO_t* fifo, void* buf, const void* head, const void* body);

/**
 * @brief Pop a single message from message fifo.
 * @param fifo Message fifo
 * @param buf Message buffer
 * @param head Message head
 * @param body Message body
 * @return Message length (num of bytes)
 */
uint32_t Msg_Pop(FIFO_t* fifo, void* buf, const void* head, void* body);

extern const MsgHead_t msg_head[];

#ifdef __cplusplus
}
#endif

#endif


