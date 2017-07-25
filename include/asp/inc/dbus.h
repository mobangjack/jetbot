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
 
#ifndef __DBUS_H__
#define __DBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "rcp.h"
#include "hcp.h"

#define DBUS_FRAME_LEN 18

/* --------------------- DBUS = RCP + HCP ------------------------ */
#define HCP_OFFSET 6

typedef struct
{
	Rcp_t rcp;
	Hcp_t hcp;
}DBus_t; // RoboMasters DBUS protocol frame structure

void DBus_Enc(const DBus_t* dbus, uint8_t* buf);
void DBus_Dec(DBus_t* dbus, const uint8_t* buf);
void DBus_Init(DBus_t* dbus);

#ifdef __cplusplus
}
#endif

#endif
