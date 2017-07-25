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

/*******************************************/
/*              Median Filter              */
/*******************************************/

#include "med.h"

void Med_Init(Med_t* med)
{
	for (med->i = 0; med->i < 3; med->i++) {
		med->buf[med->i] = 0;
		med->dst[med->i] = 0;
	}
	med->i = 0;
	med->n = 0;
	med->val = 0;
}

float Med_Proc(Med_t* med, float v)
{
	if (med->n < 2) {
		med->buf[med->i++] = v;
		med->n++;
		med->val = v;
		return med->val;
	} else {
		med->buf[med->i++] = v;
		if (med->i >= 3) med->i = 0;
		sort(med->buf, med->dst, 3);
		med->val = med->dst[1];
		return med->val;
	}
}

Med_t* Med_Create(void)
{
	Med_t* med = (Med_t*)malloc(sizeof(Med_t));
	if(med == NULL) return NULL;
	memset(med, 0, sizeof(Med_t));
	Med_Init(med);
	return med;
}

void Med_Destroy(Med_t* med)
{
	if (med != NULL) {
		free(med);
		med = NULL;
	}
}


