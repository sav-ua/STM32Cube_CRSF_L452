/*
 * ctrl_link.c
 *
 *  Created on: Aug 15, 2024
 *      Author: Andrew Sorokin
 */
#include "ctrl_link.h"

sSbusPacketStr sLinkSbusMosi;


void vCtrlLinkTransmit(sSbusPacketStr* ptr){
	  HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, 1);
	  HAL_UART_Transmit_DMA(&UART_485, (uint8_t*)ptr, sizeof(sSbusPacketStr));
}

void vTxCmplete(void){
	HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, 0);
}
