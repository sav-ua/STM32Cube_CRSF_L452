#ifndef CODEC_H_
#define CODEC_H_
#include "main.h"
#include <string.h>

//#define TEST_CRSF_TRANSMITTER


#define CRSF_PACKET_TYPE 0x16
#define EXTENDED_CRSF_PACKET_TYPE 0x64

#define CRSF_PACKET_LENGTH 26
#define CRSF_PAYLOAD_LENGTH (CRSF_PACKET_LENGTH - 4) // CRSF_PACKET_LENGTH - (header + length + type + crc)

#define EXTENDED_PAYLOAD_CNT	1
#define EXTENDED_CRSF_PACKET_LENGTH (CRSF_PAYLOAD_LENGTH * EXTENDED_PAYLOAD_CNT + 4)


#define SBUS_PACKET_LENGTH 25

#define PWM_COMPRESS_RATIO_PCT 20.0f


#define PWM_MIN_US 988.0f
#define PWM_MAX_US 2012.0f

#define PWM_MIN_CMPR_US (PWM_MIN_US + (PWM_MAX_US - PWM_MIN_US) / 100 * PWM_COMPRESS_RATIO_PCT)
#define PWM_MAX_CMPR_US (PWM_MAX_US - (PWM_MAX_US - PWM_MIN_US) / 100 * PWM_COMPRESS_RATIO_PCT)


#define TICK_PER_uS 2.4f

typedef enum{
	encoder,
	decode,
}eCodecStateEnum;


#pragma pack(1)
 typedef struct {
     // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes
     uint32_t chan0 : 11;
     uint32_t chan1 : 11;
     uint32_t chan2 : 11;
     uint32_t chan3 : 11;
     uint32_t chan4 : 11;
     uint32_t chan5 : 11;
     uint32_t chan6 : 11;
     uint32_t chan7 : 11;
     uint32_t chan8 : 11;
     uint32_t chan9 : 11;
     uint32_t chan10 : 11;
     uint32_t chan11 : 11;
     uint32_t chan12 : 11;
     uint32_t chan13 : 11;
     uint32_t chan14 : 11;
     uint32_t chan15 : 11;
 }sCrsfPayloadStr;
#pragma pack()


#pragma pack(1)
 typedef struct {
	 uint8_t header;
	 uint8_t length;
	 uint8_t type;
	 union{
		 uint8_t payload[CRSF_PAYLOAD_LENGTH];
		 sCrsfPayloadStr pl;
	 };
	 uint8_t crc;
 }sCrsfPacketStr;
#pragma pack()


#pragma pack(1)
 typedef struct {
	 uint8_t header;
	 uint8_t length;
	 uint8_t type;
	 struct{
		 union{
			 uint8_t payload0[CRSF_PAYLOAD_LENGTH];
			 sCrsfPayloadStr pl0;
		 };
/*
		 union{
			 uint8_t payload1[CRSF_PAYLOAD_LENGTH];
			 sCrsfPayloadStr pl1;
		 };

		 union{
			 uint8_t payload2[CRSF_PAYLOAD_LENGTH];
			 sCrsfPayloadStr pl2;
		 };
*/
	 }pld;
	 uint8_t crc;
 }sExtendedCrsfPacketStr;
#pragma pack()

/*
 *
Byte[0]: SBUS header, 0x0F
Byte[1 -22]: 16 servo channels, 11 bits each
Byte[23]
Bit 0: channel 17 (0x01)
Bit 1: channel 18 (0x02)
Bit 2: frame lost (0x04)
Bit 3: failsafe activated (0x08)
Byte[24]: SBUS footer
 */
#pragma pack(1)
 typedef struct {
     uint8_t chan17 			: 1;
     uint8_t chan18 			: 1;
     uint8_t frame_lost 		: 1;
     uint8_t failsafe_activated	: 1;
 }sSbusByte23Str;
#pragma pack()



#pragma pack(1)
 typedef struct {
	 uint8_t header;
	 union{
		 uint8_t payload[CRSF_PAYLOAD_LENGTH];
		 sCrsfPayloadStr pl;
	 };
	 union{
		 uint8_t b23;
		 sSbusByte23Str b23_bitfielf;
	 };
	 uint8_t footer;
 }sSbusPacketStr;
#pragma pack()


 typedef struct{
 	uint32_t uFreqCode;
 	uint32_t uFreqCodeRange;
 }sFreqCodeStr;

extern uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);
extern uint16_t uAdcData[4];

extern sFreqCodeStr vExtendedPacketCoding(uint16_t* adc,
		sCrsfPacketStr* packet,
		sExtendedCrsfPacketStr* extd_packet,
		sSbusPacketStr* sbus0,
		sSbusPacketStr* sbus1,
		eCodecStateEnum st);
#endif
