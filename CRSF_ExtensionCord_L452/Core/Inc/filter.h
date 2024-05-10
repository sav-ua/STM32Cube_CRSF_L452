/*
 * filter.h
 *
 *  Created on: 14 черв. 2023 р.
 *      Author: Andrew Sorokin
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include "main.h"
#define MOV_AVG_WIN 1500U
typedef struct{
	uint16_t arrNumbers[MOV_AVG_WIN];
	uint32_t pos;
	uint32_t sum;
}sMovAvgStr;

extern void movingAvgFlt(uint16_t* sample);
#endif /* INC_FILTER_H_ */
