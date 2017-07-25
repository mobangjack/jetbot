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

#include "jud.h"

JudFrameHeader_t* Jud_GetFrameHeader(const void* buf)
{
	JudFrameHeader_t* pheader = (JudFrameHeader_t*)buf;
	if (pheader->sof != JUD_SOF) {
		return NULL;
	}
	if (!CRC8Check(buf, JUD_HEADER_LEN, JUD_CRC8_INIT)) {
		return NULL;
	}
	return pheader;
}

void* Jud_GetData(const void* buf)
{
	const JudFrameHeader_t* pheader = (JudFrameHeader_t*)buf;
	const uint32_t frameLength = JUD_GET_FRAME_LEN(pheader->dataLength);
	if (!CRC16Check((uint8_t*)buf, frameLength, JUD_CRC16_INIT))
	{
		return NULL;
	}
	return ((uint8_t*)buf) + JUD_DATA_OFFSET;
}

JudCmdId_t Jud_GetCmdId(const void* buf)
{
	return *((JudCmdId_t*)((uint8_t*)buf + JUD_HEADER_LEN));
}





