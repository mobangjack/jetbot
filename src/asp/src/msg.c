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
 
#include "msg.h"

const MsgHead_t msg_head[] = MSG_HEAD_ARRAY;

void* Msg_GetData(const void* buf, const void* head)
{
	const MsgHead_t* headin = (MsgHead_t*)buf;
	const MsgHead_t* headex = (MsgHead_t*)head;
	uint32_t len = headex->attr.dataLength + MSG_BASE_LEN;
	if (headin->value != headex->value) {
		return NULL;
	}
	if (!CRC16Check(buf, len, headex->attr.token)) {
		return NULL;
	}
	return (((uint8_t*)buf) + sizeof(MsgHead_t));
}

uint32_t Msg_Pack(void* buf, const void* head, const void* body)
{
	uint32_t len = 0;
	const MsgHead_t* phead = (MsgHead_t*)head;
	memcpy(buf, head, sizeof(MsgHead_t));
	len += sizeof(MsgHead_t);
	memcpy((uint8_t*)buf + len, body, phead->attr.dataLength);
	len += phead->attr.dataLength;
	CRC16Append(buf, len + MSG_CRC_LEN, phead->attr.token);
	len += MSG_CRC_LEN;
	return len;
}

/**
 * @brief Push a single message to message buffer.
 * @param fifo Message fifo
 * @param buf Message buffer
 * @param head Message head
 * @param body Message body
 * @return Message length (num of bytes)
 */
uint32_t Msg_Push(FIFO_t* fifo, void* buf, const void* head, const void* body)
{
	const MsgHead_t* phead = (MsgHead_t*)head;
	uint32_t len = phead->attr.dataLength + MSG_BASE_LEN;
	if (FIFO_GetFree(fifo) < len) {
		return 0;
	}
	len = Msg_Pack(buf, head, body);
	FIFO_Push(fifo, buf, len);
	return len;
}

/**
 * @brief: Pop a single message from message buffer.
 * @param fifo Message fifo
 * @param buf Message buffer
 * @param head Message head
 * @param body Message body
 * @param Message length (num of bytes)
 */
uint32_t Msg_Pop(FIFO_t* fifo, void* buf, const void* head, void* body)
{
	MsgHead_t mhead;
	const MsgHead_t* phead = (MsgHead_t*)head;
	const uint32_t len = phead->attr.dataLength + MSG_BASE_LEN;
	if (FIFO_GetUsed(fifo) < len) {
		return 0;
	}
	FIFO_Peek(fifo, (uint8_t*)&mhead, sizeof(MsgHead_t));
	if (mhead.value != phead->value) {
		return 0;
	}
	FIFO_Peek(fifo, buf, len);
	if (!CRC16Check(buf, len, phead->attr.token)) {
		return 0;
	}
	memcpy(body, (uint8_t*)buf + sizeof(MsgHead_t), phead->attr.dataLength);
	FIFO_Pop(fifo, buf, len);
	return len;
}


