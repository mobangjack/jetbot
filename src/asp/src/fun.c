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

#include "fun.h"

/******************************************************/
/*                  Helper Functions                  */
/******************************************************/

float map(float val, float min1, float max1, float min2, float max2)
{
	LIMIT(val, min1, max1);
	return ((val-min1)*(max2-min2)/(max1-min1)+min2);
}

float constrain(float val, float min, float max)
{
	return val < min ? min : val > max ? max : val;
}

Flag_t Flag_Get(const Flag_t* flag, Flag_t mask)
{
	return (*flag) & mask;
}

Flag_t Flag_Hit(const Flag_t* flag, Flag_t mask)
{
	return (((*flag) & mask) == mask);
}

void Flag_Set(Flag_t* flag, Flag_t mask)
{
	(*flag) |= mask;
}

void Flag_Clr(Flag_t* flag, Flag_t mask)
{
	(*flag) &= ~mask;
}

void Flag_Tog(Flag_t* flag, Flag_t mask)
{
	Flag_Get(flag, mask) ? Flag_Clr(flag, mask) : Flag_Set(flag, mask);
}

void Flag_Cpy(Flag_t* flag, Flag_t src, Flag_t mask)
{
	uint8_t n = sizeof(Flag_t) * 8;
	uint8_t i = 0;
	Flag_t tmp = 0;
	for (i = 0; i < n; i++) {
		tmp =  (((uint32_t)0x01) << i) & mask;
		if (Flag_Get(&src, tmp)) {
			Flag_Set(flag, tmp);
		} else {
			Flag_Clr(flag, tmp);
		}
	}
}

void Flag_Det(Flag_t* flag, Flag_t mask, uint32_t condition)
{
	if (condition) {
		Flag_Set(flag, mask);
	} else {
		Flag_Clr(flag, mask);
	}
}

void isort(float* dst, uint32_t len, float val)
{
	uint32_t i = 0;
	for (; i < len; i++) {
		if (val < dst[i]) {
			uint32_t j = len;
			for (; j > i; j--) {
				dst[j] = dst[j-1];
			}
			break;
		}
	}
	dst[i] = val;
}

void sort(const float* arr, float* dst, uint32_t len)
{
	uint32_t i = 1;
	dst[0] = arr[0];
	for (; i < len; i++) {
		isort(dst, i, arr[i]);
	}
}

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

