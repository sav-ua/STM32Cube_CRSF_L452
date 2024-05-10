#include "filter.h"



sMovAvgStr sMovAvg[4] = {
		{
				.arrNumbers = {0},
				.pos = 0,
				.sum = 0,
		},
		{
				.arrNumbers = {0},
				.pos = 0,
				.sum = 0,
		},
		{
				.arrNumbers = {0},
				.pos = 0,
				.sum = 0,
		},
		{
				.arrNumbers = {0},
				.pos = 0,
				.sum = 0,
		},
};


static uint16_t movingAvg(uint16_t* ptrArrNumbers, uint32_t *ptrSum, uint32_t* pos, uint16_t nextNum){
	*ptrSum = *ptrSum - ptrArrNumbers[*pos] + nextNum;	//Subtract the oldest number from the prev sum, add the new number
	ptrArrNumbers[*pos] = nextNum;						//Assign the nextNum to the position in the array
	if (++*pos >= MOV_AVG_WIN) *pos = 0;
	return *ptrSum / MOV_AVG_WIN;						//return the average
}

void movingAvgFlt(uint16_t* sample){
	for(int i = 0; i < 4; i++){
		sample[i] = movingAvg(sMovAvg[i].arrNumbers, &sMovAvg[i].sum, &sMovAvg[i].pos, sample[i]);
	}
}

