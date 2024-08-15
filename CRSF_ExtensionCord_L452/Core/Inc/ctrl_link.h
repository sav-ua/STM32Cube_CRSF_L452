/*
 * ctrl_link.h
 *
 *  Created on: Aug 15, 2024
 *      Author: Andrew Sorokin
 */

#ifndef INC_CTRL_LINK_H_
#define INC_CTRL_LINK_H_


#include "main.h"
#include "codec.h"

extern UART_HandleTypeDef huart3;


extern sSbusPacketStr sLinkSbusMosi;

extern void vTxCmplete(void);
extern void vCtrlLinkTransmit(sSbusPacketStr* ptr);
#endif /* INC_CTRL_LINK_H_ */
