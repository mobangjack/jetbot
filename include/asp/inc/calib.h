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
 
#ifndef __CALIB_H__
#define __CALIB_H__

/*****************************************/
/*              Calibration              */
/*****************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#pragma pack(1)
	
#define PID_CALIB_TYPE_CHASSIS_VELOCITY 0x01
#define PID_CALIB_TYPE_GRABBER_VELOCITY 0x02
#define PID_CALIB_TYPE_GRABBER_POSITION 0x03
#define PID_CALIB_VALUE_SCALE 1.0f
typedef struct
{
	uint8_t type;
	uint16_t kp;
	uint16_t ki;
	uint16_t kd;
	uint16_t db; // Dead band, cut off output when abs error < db
	uint16_t it; // Integral threshold, compute integral when abs error < it
	uint16_t Emax; // Error max
	uint16_t Pmax; // Component P output max
	uint16_t Imax; // Component I output max
	uint16_t Dmax; // Component D output max
	uint16_t Omax;
}PIDCalib_t; // PID Calibration

typedef struct
{
	float kp;
	float ki;
	float kd;
	float db; // Dead band, cut off output when abs error < db
	float it; // Integral threshold, compute integral when abs error < it
	float Emax; // Error max
	float Pmax; // Component P output max
	float Imax; // Component I output max
	float Dmax; // Component D output max
	float Omax;
}PIDParam_t; // PID Parameters

#define IMU_CALIB_VALUE_SCALE 1.0f
typedef struct
{
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;
	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
}IMUCalib_t; // IMU offset Calibration

typedef struct
{
	float ax_offset;
	float ay_offset;
	float az_offset;
	float gx_offset;
	float gy_offset;
	float gz_offset;
}IMUParam_t; // IMU Parameters

#define MAG_CALIB_VALUE_SCALE 1.0f
typedef struct
{
	int16_t mx_offset;
	int16_t my_offset;
	int16_t mz_offset;
}MagCalib_t; // Mag offset Calibration

typedef struct
{
	float mx_offset;
	float my_offset;
	float mz_offset;
}MagParam_t; // Mag offset Parameters

#define SYS_CALIB_VALUE_SCALE 1e3f
typedef struct
{
	uint16_t x; //
	uint16_t y; //
	uint16_t z; //
	uint16_t p; //
	uint16_t t; //
}SysCalib_t; // System Calibration

typedef struct
{
	float x; //
	float y; //
	float z; //
	float p; //
	float t; //
}SysParam_t; // System Parameters

#define VEL_CALIB_VALUE_SCALE SYS_CALIB_VALUE_SCALE
typedef SysCalib_t VelCalib_t; // Velocity Calibration
typedef SysParam_t VelParam_t; // Velocity Parameters

/*******************************************/
/*     Coordinate Transforming System      */
/* Mecanum Wheel Power Transmission System */
/*******************************************/
/*              2        1                 */
/*                  |y                     */
/*                  |___x                  */
/*               z                         */
/*              3        4                 */
/*                                         */
/*******************************************/
#define MEC_CALIB_VALUE_SCALE 1e3f
typedef struct
{
	uint16_t lx; // mm
	uint16_t ly; // mm
	uint16_t r1; // mm
	uint16_t r2; // mm
}MecCalib_t; // Mecanum Wheel Calibration

typedef struct
{
	float lx; // m
	float ly; // m
	float r1; // m
	float r2; // m
}MecParam_t; // Mecanum Wheel Parameters

#define POS_CALIB_VALUE_SCALE 1e3f
typedef struct
{
	int16_t p; //
	int16_t t; //
}PosCalib_t; // Position Calibration

typedef struct
{
	float p; //
	float t; //
}PosParam_t; // Position Parameters

typedef struct
{
	PIDCalib_t cvl; // Chasis velocity loop calibration
	PIDCalib_t gvl; // Gimbal velocity loop calibration
	PIDCalib_t gpl; // Chasis position loop calibration
	IMUCalib_t imu; // IMU offset calibration
	MagCalib_t mag; // Mag offset calibration
	MecCalib_t mec; // Mecanum wheel calibration
	PosCalib_t pos; // Position limit calibration
}Calib_t; // Calibration

typedef struct
{
	PIDParam_t cvl; // Chasis velocity loop parameters
	PIDParam_t gvl; // Gimbal velocity loop parameters
	PIDParam_t gpl; // Chasis position loop parameters
	IMUParam_t imu; // IMU calibration
	MagParam_t mag; // Mag calibration
	MecParam_t mec; // Mecanum wheel calibration
	PosParam_t pos; // Position calibration
}Param_t; // Parameters

#pragma pack()

void Calib_GetPID(PIDCalib_t* PIDCalib, const PIDParam_t* PIDParam);
void Calib_SetPID(PIDParam_t* PIDParam, const PIDCalib_t* PIDCalib);
void Calib_GetIMU(IMUCalib_t* IMUCalib, const IMUParam_t* IMUParam);
void Calib_SetIMU(IMUParam_t* IMUParam, const IMUCalib_t* IMUCalib);
void Calib_GetMag(MagCalib_t* MagCalib, const MagParam_t* MagParam);
void Calib_SetMag(MagParam_t* MagParam, const MagCalib_t* MagCalib);
void Calib_GetSys(SysCalib_t* SysCalib, const SysParam_t* SysParam);
void Calib_SetSys(SysParam_t* SysParam, const SysCalib_t* SysCalib);
void Calib_GetVel(VelCalib_t* VelCalib, const VelParam_t* VelParam);
void Calib_SetVel(VelParam_t* VelParam, const VelCalib_t* VelCalib);
void Calib_GetMec(MecCalib_t* MecCalib, const MecParam_t* MecParam);
void Calib_SetMec(MecParam_t* MecParam, const MecCalib_t* MecCalib);
void Calib_GetPos(PosCalib_t* PosCalib, const PosParam_t* PosParam);
void Calib_SetPos(PosParam_t* PosParam, const PosCalib_t* PosCalib);

#ifdef __cplusplus
}
#endif

#endif




