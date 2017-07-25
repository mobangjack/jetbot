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

#ifndef __MED_H__
#define __MED_H__

/*******************************************/
/*              Median Filter              */
/*******************************************/

#ifdef __cpluplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "fun.h"

typedef struct
{
	float buf[3];  // ring buffer
	float dst[3];  // sorted buffer
	uint8_t n;     // number of init data
	uint8_t i;     // index of ring buffer
	float val;     // median
}Med_t;

void Med_Init(Med_t* med);
float Med_Proc(Med_t* med, float v);

Med_t* Med_Create(void);
void Med_Destroy(Med_t* med);

#ifdef __cpluplus
}
#endif

#endif


