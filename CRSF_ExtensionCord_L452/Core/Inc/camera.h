/*
 * camera.h
 *
 *  Created on: Nov 8, 2023
 *      Author: Andrew Sorokin
 */

#ifndef INC_CAMERA_H_
#define INC_CAMERA_H_
#include "main.h"
//#include "usart.h"
#include "string.h"
#include "codec.h"

extern UART_HandleTypeDef hlpuart1;
/*
  Name							   | CMD   |    Length|
------------------------------------------------------
Auto focus 							0x04 		1
Manual Zoom and Auto Focus			0x05		1
Manual Focus						0x06		1
Gimbal Rotation						0x07		2			p.37
Center								0x08		1
Photo and Video						0x0C		1
Set Camera Image Type				0x11		1
 */

typedef enum{
	Auto_focus 				= 0x04,
	Manual_Zoom				= 0x05,
	Manual_Focus			= 0x06,
	Gimbal_Rotation			= 0x07,
	Center					= 0x08,
	Photo_and_Video			= 0x0C,
//	Set_Camera_Image_Type	= 0x11,
	Send_Image_Mode_to_Gimbal_Camera	= 0x11,
	Send_Thermal_Color_to_Gimbal_Camera	= 0x1B,//p59
//	Send_Codec_Specs_to_Gimbal_Camera 	= 0x21,//p55

}eCameraCmdStr;



#pragma pack(1)
typedef struct{
	uint16_t	stx;			// 0x6655 starting mark. Low byte in the front
	uint8_t		ctrl;			// 0 need_ack (if the current data pack need “ack”). 1 ack_pack (if the current data pack is an “ack” package)
	uint16_t	data_len;		// Date field byte length. Low byte in the front
	uint16_t	seq;			// Frame sequence (0 ~ 65535). Low byte in the front
	uint8_t		cmd_id;			// Command ID
	uint8_t		data;			// Data
	uint16_t	crc;			// CRC16 check to the complete data package. Low byte in the front
}sCameraPacketL1Str;
#pragma pack()

#pragma pack(1)
typedef struct{
	uint16_t	stx;			// 0x6655 starting mark. Low byte in the front
	uint8_t		ctrl;			// 0 need_ack (if the current data pack need “ack”). 1 ack_pack (if the current data pack is an “ack” package)
	uint16_t	data_len;		// Date field byte length. Low byte in the front
	uint16_t	seq;			// Frame sequence (0 ~ 65535). Low byte in the front
	uint8_t		cmd_id;			// Command ID
	uint8_t		data1;			// Data 1
	uint8_t		data2;			// Data 2
	uint16_t	crc;			// CRC16 check to the complete data package. Low byte in the front
}sCameraPacketL2Str;
#pragma pack()

#pragma pack(1)
typedef struct{
	uint16_t	stx;			// 0x6655 starting mark. Low byte in the front
	uint8_t		ctrl;			// 0 need_ack (if the current data pack need “ack”). 1 ack_pack (if the current data pack is an “ack” package)
	uint16_t	data_len;		// Date field byte length. Low byte in the front
	uint16_t	seq;			// Frame sequence (0 ~ 65535). Low byte in the front
	uint8_t		cmd_id;			// Command ID
	uint8_t		data1;			// Data 1
	uint8_t		data2;			// Data 2
	uint16_t	data3;			// Data 3
	uint16_t	data4;			// Data 4
	uint16_t	data5;			// Data 5
	uint8_t		data6;			// Data 6
	uint16_t	crc;			// CRC16 check to the complete data package. Low byte in the front
}sCameraPacketL6Str;
#pragma pack()


typedef void(*vCameraFuncPrtType)(sSbusPacketStr* p);

extern sCameraPacketL1Str sCameraPacketL1;
extern sCameraPacketL2Str sCameraPacketL2;

extern void vCameraPacketComposeL2(sCameraPacketL2Str* packet, eCameraCmdStr cmd, int8_t param1, int8_t param2);
extern void vCameraPacketComposeL1(sCameraPacketL1Str* packet, eCameraCmdStr cmd, int8_t param);
extern void vConvertSbusToCameraProtocol(sSbusPacketStr* p);

#endif /* INC_CAMERA_H_ */
