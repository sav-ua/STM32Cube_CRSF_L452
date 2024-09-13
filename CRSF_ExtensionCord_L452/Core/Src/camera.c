/*
 * camera.c
 *
 *  Created on: Nov 8, 2023
 *      Author: Andrew Sorokin
 */

#include "camera.h"
#include "codec.h"

const uint16_t crc16_tab[256]={
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0

};

// 11 bytes
sCameraPacketL1Str sCameraPacketL1 = {//Auto Centering  {55 66 01 01 00 00 00 08 01 d1 12}
		.stx		=	0x6655,
		.ctrl		=	0x01,
		.data_len	=	0x0001,
		.seq		=	0x0000,
		.cmd_id		=	0x08,
		.data		=	0x01,
		.crc		=	0x0000
};
sCameraPacketL2Str sCameraPacketL2 = {
		.stx		=	0x6655,
		.ctrl		=	0x01,
		.data_len	=	0x0001,
		.seq		=	0x0000,
		.cmd_id		=	0x08,
		.data1		=	0x01,
		.data2		=	0x02,
		.crc		=	0x0000
};

sCameraPacketL6Str sCameraPacketL6 = {
		.stx		=	0x6655,
		.ctrl		=	0x01,
		.data_len	=	0x0009,
		.seq		=	0x0000,
		.cmd_id		=	0x21,
		.data1		=	0x01,
		.data2		=	0x02,
		.data3		=	1280,
		.data4		=	720,
		.data5		=	1500,
		.data6		=	0x00,
		.crc		=	0x0000
};


/***********************************************************
CRC16 Coding & Decoding G(X) = X^16+X^12+X^5+1
***********************************************************/
static uint16_t CRC16_cal(uint8_t *ptr, uint32_t len, uint16_t crc_init){
	uint16_t crc, oldcrc16;
	uint8_t temp;
		crc = crc_init;
		while (len -- != 0){
			temp = (crc >> 8) & 0xff;
			oldcrc16 = crc16_tab[*ptr^temp];
			crc = (crc << 8) ^ oldcrc16;
			ptr++;
		}
		//crc=~crc;
		return(crc);
}
static uint8_t crc_check_16bites(uint8_t* pbuf, uint32_t len, uint16_t* p_result){
	uint16_t crc_result = 0;
	crc_result= CRC16_cal(pbuf, len, 0);
	*p_result = crc_result;
	return 2;
}

void vCameraPacketComposeL2(sCameraPacketL2Str* packet, eCameraCmdStr cmd, int8_t param1, int8_t param2){
	packet->stx			=	0x6655;
	packet->ctrl		=	0x01;
	packet->data_len	=	0x0002;
	packet->seq			=	0x0000;
	packet->cmd_id		=	cmd;
	packet->data1		=	*(uint8_t*)&param1;
	packet->data2		=	*(uint8_t*)&param2;
	crc_check_16bites((uint8_t*)packet, sizeof(sCameraPacketL2Str) - 2, &packet->crc);
}
void vCameraPacketComposeL1(sCameraPacketL1Str* packet, eCameraCmdStr cmd, int8_t param){
	packet->stx			=	0x6655;
	packet->ctrl		=	0x01;
	packet->data_len	=	0x0001;
	packet->seq			=	0x0000;
	packet->cmd_id		=	cmd;
	packet->data		=	*(uint8_t*)&param;
	crc_check_16bites((uint8_t*)packet, sizeof(sCameraPacketL1Str) - 2, &packet->crc);
}
void vCameraPacketComposeL6(sCameraPacketL6Str* packet, eCameraCmdStr cmd, int8_t param){
	packet->stx			=	0x6655;
	packet->ctrl		=	0x01;
	packet->data_len	=	0x0009;
	packet->seq			=	0x0000;
	packet->cmd_id		=	cmd;
	packet->data1		=	*(uint8_t*)&param;
	crc_check_16bites((uint8_t*)packet, sizeof(sCameraPacketL6Str) - 2, &packet->crc);
}



#define RC_SW_CH_MIN_L		111
#define RC_SW_CH_MIN_H		351
#define RC_SW_CH_MIDDLE_L	911
#define RC_SW_CH_MIDDLE_H	1071
#define RC_SW_CH_MAX_L		1711
#define RC_SW_CH_MAX_H		1871


#define RC_STICK_CH_MIN			172
#define RC_STICK_CH_MIDDLE_L	943
#define RC_STICK_CH_MIDDLE_H	1039
#define RC_STICK_CH_MAX			1811
#define RC_STICK_CH_LOW_SCF		100.f / (float)(RC_STICK_CH_MIDDLE_L - RC_STICK_CH_MIN)	// Scale factor from (RC_STICK_CH_MIDDLE_L - RC_STICK_CH_MIN) --> 100
#define RC_STICK_CH_HIGH_SCF	100.f / (float)(RC_STICK_CH_MAX - RC_STICK_CH_MIDDLE_H)	// Scale factor from (RC_STICK_CH_MIDDLE_H - RC_STICK_CH_MIN) --> 100


#define RC_SW_LEVEL0_MIN 938
#define RC_SW_LEVEL0_MAX 1038

#define RC_SW_LEVEL1_MIN 1143
#define RC_SW_LEVEL1_MAX 1243

#define RC_SW_LEVEL2_MIN 1348
#define RC_SW_LEVEL2_MAX 1448

#define RC_SW_LEVEL3_MIN 1552
#define RC_SW_LEVEL3_MAX 1652

#define RC_SW_LEVEL4_MIN 1757
#define RC_SW_LEVEL4_MAX 1857

#define RC_SW_LEVEL5_MIN 1962
#define RC_SW_LEVEL5_MAX 2062

/*----------------------------------------------------------------------------------------------
Switch channel
________________PWM (us)________________RC value_____
min		|	950us	-	1100us	|	111		351
mid		|	1450us	-	1550us	|	911		1071
high	|	1950us	-	2050us	|	1711	1871
-----------------------------------------------------

Stick channel
________________PWM (us)________________RC value_____
min		|			988us		|		172
mid		|	1470us	-	1530us	|	943		1039
high	|			2012us		|		1811
------------------------------------------------------



//-------------------------------
Sbus
0 yaw
Left   988
neutral 1470-1530
Right  2012

1 pitch
Down   988
neutral 1470-1530
UP     2012

2 zoom
 ---    988
 neutral 1470-1530
 +++    2012
3 center
 OFF 950-1100
 ON  1950-2050

4 video rec
 OFF 950-1100
 ON  1950-2050

5 gimbal_motion_mode
 FPV Mode    950-1100
 Follow Mode 1450-1550
 Lock Mode   1950-2050

6 auto focus
 OFF 950-1100
 ON  1950-2050

7 manual focus
 min 988
 mid 1470-1530
 higt 2012
------------------------------------------------  15.06.2024 and ZT6 camera compatibility -----------
8 Request the Thermal Color Palette
 0	938-1038		White_Hot 	0
 1	1043-1143		Ironbow		3
 2	1348-1448		Night		5
 3	1552-1652		Red_Hot		7
 4	1757-1857		Black_Hot	10
 5	1962-2062		Glory_Hot	11

//---------------------------------------------------------------------------------------*/
static void vCameraGimbalRotaionCtrl (sSbusPacketStr* p){
	static uint16_t ch0_old = 0;
	static uint16_t ch1_old = 0;
	int8_t  yaw, pitch;
		if((p->pl.chan0 == ch0_old) && (p->pl.chan1 == ch1_old))
			return; // Exit if no changes
		else{
			ch0_old = p->pl.chan0;
			ch1_old = p->pl.chan1;
		}
		//Gimbal Rotation - YAW
		if(p->pl.chan0 < RC_STICK_CH_MIDDLE_L){
			if(p->pl.chan0 <= RC_STICK_CH_MIN) p->pl.chan0 = RC_STICK_CH_MIN;
			yaw = ((int8_t)((float)(RC_STICK_CH_MIDDLE_L - p->pl.chan0) * RC_STICK_CH_LOW_SCF)) * (-1);
		}
		else if((p->pl.chan0 >= RC_STICK_CH_MIDDLE_L) &&  (p->pl.chan0 <= RC_STICK_CH_MIDDLE_H)){
			yaw = 0;
		}
		else if(p->pl.chan0 > RC_STICK_CH_MIDDLE_H){
			if(p->pl.chan0 >= RC_STICK_CH_MAX) p->pl.chan0 = RC_STICK_CH_MAX;
			yaw = ((int8_t)((float)(p->pl.chan0 - RC_STICK_CH_MIDDLE_H) * RC_STICK_CH_HIGH_SCF));
		}
		else yaw = 0;


		//Gimbal Rotation - PITCH
		if(p->pl.chan1 < RC_STICK_CH_MIDDLE_L){
			if(p->pl.chan1 <= RC_STICK_CH_MIN) p->pl.chan1 = RC_STICK_CH_MIN;
			pitch = ((int8_t)((float)(RC_STICK_CH_MIDDLE_L - p->pl.chan1) * RC_STICK_CH_LOW_SCF)) * (-1);
		}
		else if((p->pl.chan1 >= RC_STICK_CH_MIDDLE_L) &&  (p->pl.chan1 <= RC_STICK_CH_MIDDLE_H)){
			pitch = 0;
		}
		else if(p->pl.chan1 > RC_STICK_CH_MIDDLE_H){
			if(p->pl.chan1 >= RC_STICK_CH_MAX) p->pl.chan1 = RC_STICK_CH_MAX;
			pitch = ((int8_t)((float)(p->pl.chan1 - RC_STICK_CH_MIDDLE_H) * RC_STICK_CH_HIGH_SCF));
		}
		else pitch = 0;

		vCameraPacketComposeL2(&sCameraPacketL2, Gimbal_Rotation, yaw, pitch);
		HAL_UART_Transmit_DMA(&UART_CAM_CTRL, (uint8_t*)&sCameraPacketL2, sizeof(sCameraPacketL2));
		HAL_Delay(1);
}
static void vCameraManualZoomAndAutoFocus(sSbusPacketStr* p){
	static uint16_t ch2_old = 0;
	int8_t  zoom;
		if(p->pl.chan2 == ch2_old)
			return; // Exit if no changes
		else	ch2_old = p->pl.chan2;
		if		(p->pl.chan2 < RC_SW_CH_MIN_H)												zoom= -1;
		else if((p->pl.chan2 >= RC_SW_CH_MIDDLE_L) && (p->pl.chan2 <= RC_SW_CH_MIDDLE_H))	zoom = 0;
		else if (p->pl.chan2 > RC_SW_CH_MAX_L)												zoom = 1;
		else return;
		vCameraPacketComposeL1(&sCameraPacketL1, Manual_Zoom, zoom);
		HAL_UART_Transmit_DMA(&UART_CAM_CTRL, (uint8_t*)&sCameraPacketL1, sizeof(sCameraPacketL1));
		HAL_Delay(1);
}
static void vCameraCenter(sSbusPacketStr* p){
	static uint16_t ch3_old = 0;
	int8_t  center;
		if(p->pl.chan3 == ch3_old)
			return; // Exit if no changes
		else	ch3_old = p->pl.chan3;
		if (p->pl.chan3 > RC_SW_CH_MAX_L){
			center = 1;
			vCameraPacketComposeL1(&sCameraPacketL1, Center, center);
			HAL_UART_Transmit_DMA(&UART_CAM_CTRL, (uint8_t*)&sCameraPacketL1, sizeof(sCameraPacketL1));
			HAL_Delay(1);
		}
}
static void vCameraPhotoAndVideoRecording(sSbusPacketStr* p){
	static uint16_t ch4_old = 0;
	int8_t  rec;
		if(p->pl.chan4 == ch4_old)
			return; // Exit if no changes
		else	ch4_old = p->pl.chan4;
		if (p->pl.chan4 > RC_SW_CH_MAX_L){
			rec = 2;
			vCameraPacketComposeL1(&sCameraPacketL1, Photo_and_Video, rec);
			HAL_UART_Transmit_DMA(&UART_CAM_CTRL, (uint8_t*)&sCameraPacketL1, sizeof(sCameraPacketL1));
			HAL_Delay(1);
		}
}
static void vCameraPhotoAndVideoMotionMode(sSbusPacketStr* p){
	static uint16_t ch5_old = 0;
	int8_t  motion;
		if(p->pl.chan5 == ch5_old)
			return; // Exit if no changes
		else	ch5_old = p->pl.chan5;
		if		(p->pl.chan5 < RC_SW_CH_MIN_H)												motion 	= 3;	// FPV Mode
		else if((p->pl.chan5 >= RC_SW_CH_MIDDLE_L) && (p->pl.chan5 <= RC_SW_CH_MIDDLE_H))	motion  = 4;	// Follow Mode
		else if (p->pl.chan5 > RC_SW_CH_MAX_L)												motion  = 5;	// Lock Mode
		else return;
		vCameraPacketComposeL1(&sCameraPacketL1, Photo_and_Video, motion);
		HAL_UART_Transmit_DMA(&UART_CAM_CTRL, (uint8_t*)&sCameraPacketL1, sizeof(sCameraPacketL1));
		HAL_Delay(1);
}
static void vCameraAutoFocus(sSbusPacketStr* p){
	static uint16_t ch6_old = 0;
	int8_t  start;
		if(p->pl.chan6 == ch6_old)
			return; // Exit if no changes
		else	ch6_old = p->pl.chan6;
		if (p->pl.chan6 > RC_SW_CH_MAX_L){
			start = 1;
			vCameraPacketComposeL1(&sCameraPacketL1, Auto_focus, start);
			HAL_UART_Transmit_DMA(&UART_CAM_CTRL, (uint8_t*)&sCameraPacketL1, sizeof(sCameraPacketL1));
			HAL_Delay(1);
		}
}
static void vCameraManualFocus(sSbusPacketStr* p){
	static uint16_t ch7_old = 0;
	int8_t  m_focus;
		if(p->pl.chan7 == ch7_old)
			return; // Exit if no changes
		else	ch7_old = p->pl.chan7;
		if		(p->pl.chan7 < RC_SW_CH_MIN_H)												m_focus	= -1;	// FPV Mode
		else if((p->pl.chan7 >= RC_SW_CH_MIDDLE_L) && (p->pl.chan7 <= RC_SW_CH_MIDDLE_H))	m_focus	= 0;	// Follow Mode
		else if (p->pl.chan7 > RC_SW_CH_MAX_L)												m_focus	= 1;	// Lock Mode
		else return;
		vCameraPacketComposeL1(&sCameraPacketL1, Manual_Focus, m_focus);
		HAL_UART_Transmit_DMA(&UART_CAM_CTRL, (uint8_t*)&sCameraPacketL1, sizeof(sCameraPacketL1));
		HAL_Delay(1);
}

//------------------------------------------------  15.06.2024 and ZT6 camera compatibility -----------

static void vSend_Thermal_Color_to_Gimbal_Camera(sSbusPacketStr* p){
	static uint16_t ch8_old = 0;
	int8_t  palette;
		if(p->pl.chan8 == ch8_old)
			return; // Exit if no changes
		else	ch8_old = p->pl.chan8;
		if		((p->pl.chan8 >= convert_us_to_channel_value(RC_SW_LEVEL0_MIN)) && (p->pl.chan8 <= convert_us_to_channel_value(RC_SW_LEVEL0_MAX)))	palette	= 0; 	//White_Hot 0
		else if	((p->pl.chan8 >= convert_us_to_channel_value(RC_SW_LEVEL1_MIN)) && (p->pl.chan8 <= convert_us_to_channel_value(RC_SW_LEVEL1_MAX)))	palette	= 3;  	//Ironbow	3
		else if	((p->pl.chan8 >= convert_us_to_channel_value(RC_SW_LEVEL2_MIN)) && (p->pl.chan8 <= convert_us_to_channel_value(RC_SW_LEVEL2_MAX)))	palette	= 5;  	//Night		5
		else if	((p->pl.chan8 >= convert_us_to_channel_value(RC_SW_LEVEL3_MIN)) && (p->pl.chan8 <= convert_us_to_channel_value(RC_SW_LEVEL3_MAX)))	palette	= 7;  	//Red_Hot	7
		else if	((p->pl.chan8 >= convert_us_to_channel_value(RC_SW_LEVEL4_MIN)) && (p->pl.chan8 <= convert_us_to_channel_value(RC_SW_LEVEL4_MAX)))	palette	= 10;  	//Black_Hot	10
		else if	((p->pl.chan8 >= convert_us_to_channel_value(RC_SW_LEVEL5_MIN)) && (p->pl.chan8 <= convert_us_to_channel_value(RC_SW_LEVEL5_MAX)))	palette	= 11;  	//Glory_Hot	11
		else return;
		vCameraPacketComposeL1(&sCameraPacketL1, Send_Thermal_Color_to_Gimbal_Camera, palette);
		HAL_UART_Transmit_DMA(&UART_CAM_CTRL, (uint8_t*)&sCameraPacketL1, sizeof(sCameraPacketL1));
		HAL_Delay(1);
}

static void vSend_Image_Mode_to_Gimbal_Camera(sSbusPacketStr* p){
	static uint16_t ch10_old = 0;
	int8_t  iCodecSpecs = 0;

		if(p->pl.chan10 == ch10_old)
			return; // Exit if no changes
		else	ch10_old = p->pl.chan10;

		if		(p->pl.chan10 < RC_SW_CH_MIN_H)												iCodecSpecs = 0;	//
		else if((p->pl.chan10 >= RC_SW_CH_MIDDLE_L) && (p->pl.chan10 <= RC_SW_CH_MIDDLE_H))	iCodecSpecs = 7;	//
		else if (p->pl.chan10 > RC_SW_CH_MAX_L)												iCodecSpecs = 3;	//
		else return;

		vCameraPacketComposeL1(&sCameraPacketL1, Send_Image_Mode_to_Gimbal_Camera, iCodecSpecs);
		HAL_UART_Transmit_DMA(&UART_CAM_CTRL, (uint8_t*)&sCameraPacketL1, sizeof(sCameraPacketL1));
		HAL_Delay(6000);

		vCameraPacketComposeL1(&sCameraPacketL1, Photo_and_Video, 7);
		HAL_UART_Transmit_DMA(&UART_CAM_CTRL, (uint8_t*)&sCameraPacketL1, sizeof(sCameraPacketL1));
		HAL_Delay(1);

}


vCameraFuncPrtType vCameraFuncArray[] = {
		&vCameraGimbalRotaionCtrl,
		&vCameraManualZoomAndAutoFocus,
		&vCameraCenter,
		&vCameraPhotoAndVideoRecording,
		&vCameraPhotoAndVideoMotionMode,
		&vCameraAutoFocus,
		&vCameraManualFocus,
		&vSend_Image_Mode_to_Gimbal_Camera,
		&vSend_Thermal_Color_to_Gimbal_Camera,
};
void vConvertSbusToCameraProtocol(sSbusPacketStr* p){
	static uint32_t c = 0;
	(*vCameraFuncArray[c])(p);
	if(++c >= sizeof(vCameraFuncArray) / sizeof(vCameraFuncArray[0])) c = 0;
}
