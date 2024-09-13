/*
 * ctrl_link.c
 *
 *  Created on: Aug 15, 2024
 *      Author: Andrew Sorokin
 */
#include "ctrl_link.h"



void vCtrlLinkTransmitRC_Side(sExtendedCrsfPacketStr* ptr){				// Transmit routine from Remote Control (RC) side
	if(uGetCorddSide() == rc_side){
	  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, 1);
	  HAL_UART_Transmit_DMA(&UART_485, (uint8_t*)ptr, sizeof(sExtendedCrsfPacketStr));
	}
}

void vTxCmplete(void){
	HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, 0);
}
eCordSideEnum uGetCorddSide(void){
	if(HAL_GPIO_ReadPin(MVIDEO_GPIO_Port, MVIDEO_Pin))
		return rf_side;
	else
		return rc_side;
}
#if 0
void uSetCorddSide(eCordSideEnum s){
	if(s == rc_side){

	}
	else if(s == rf_side){

	}
	else
		return;
}
#endif
