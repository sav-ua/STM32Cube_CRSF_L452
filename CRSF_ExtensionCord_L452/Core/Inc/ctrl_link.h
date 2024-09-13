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


typedef enum{
	rc_side = 0,
	rf_side,
}eCordSideEnum;

extern UART_HandleTypeDef huart3;

extern eCordSideEnum uGetCorddSide(void);
extern void vTxCmplete(void);
extern void vCtrlLinkTransmitRC_Side(sExtendedCrsfPacketStr* ptr);
#endif /* INC_CTRL_LINK_H_ */
