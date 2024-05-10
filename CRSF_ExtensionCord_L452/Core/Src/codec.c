#include "codec.h"
/*
crsf_RC_packed_t_str sVltgToCrsfPacket = {
		.header = 0xEE,
		.length = 0x18, //24d
		.type = 0x16
};
*/


uint16_t uAdcData[4] = {2048, 2048, 2048, 2048};

static inline uint16_t convert_code_to_channel_value(uint16_t code){
     /*
      *       RC     ADC
      * min  172 <-  0
      * mid  992 <- 2048
      * max 1811 <- 4096
      */
	static const float scale = (1811.f - 172.f) / (4096.f - 0.f);
	static const float offset = (172.f);
		return (scale * code) + offset;
 }

static uint16_t convert_us_to_channel_value(uint16_t pwm_us){
//#define PWM_MIN_US 988.0f
//#define PWM_MAX_US 2012.0f
	/*
	*      PWM   ->   RC
	* min  988us ->  172
	* mid 1500us ->  992
	* max 2012us -> 1811
	*/
	static const float scale = (1811.f - 172.f) / (PWM_MAX_US - PWM_MIN_US);
	static const float offset = 172.f - PWM_MIN_US * scale;
		return (scale * pwm_us) + offset;
}

static inline uint16_t convert_channel_value_to_us(uint16_t chan_value){
	/*
	*       RC     PWM
	* min  172 ->  988us
	* mid  992 -> 1500us
	* max 1811 -> 2012us
	*/
	static const float scale = (PWM_MAX_US - PWM_MIN_US) / (1811.f - 172.f);
	static const float offset = PWM_MIN_US - 172.f * scale;
		return (scale * chan_value) + offset;
}

static inline uint16_t convert_channel_value_to_us_compressed(uint16_t chan_value){ //This routine compresses the original range to 20..80%
	// Original range from 988us to 2012us = 1024
	// 20% from 1024 = 204.8 -> min = 988 + 205 = 1193us -> max = 2012 - 205 = 1807us
	/*
	*       RC     PWM
	* min  172 ->  1193us
	* mid  992 -> 1500us
	* max 1811 -> 1807us
	*/
	static const float scale = (PWM_MAX_CMPR_US - PWM_MIN_CMPR_US) / (1811.f - 172.f);
	static const float offset = PWM_MIN_CMPR_US - 172.f * scale;
		return (scale * chan_value) + offset;
}


uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a){
     crc ^= a;
     for (int i = 0; i < 8; ++i){
         if (crc & 0x80)	crc = (crc << 1) ^ 0xD5;
         else				crc = crc << 1;
     }
     return crc;
}

sFreqCodeStr vExtendedPacketCoding(uint16_t* adc,
		sCrsfPacketStr* packet,							// Incoming CRSF packet from Remote control
		sExtendedCrsfPacketStr* extd_packet,			// Extended CRSF packet is used for communication between the uCUs on both sides
		sSbusPacketStr* sbus0,							// Incoming SBUS packet from Remote control (Camera control)
		sSbusPacketStr* sbus1,							// Outgoing SBUS packet from RF side (Camera control)
		eCodecStateEnum st){
	static sFreqCodeStr f = {1000, 1000};
	static uint32_t codec_address_offset = 0;
		if(st == encoder){
			extd_packet->header			= 0xEE;
			extd_packet->length			= CRSF_PACKET_LENGTH - 2/*EXTENDED_CRSF_PACKET_LENGTH - 2*/;
			extd_packet->type			= EXTENDED_CRSF_PACKET_TYPE + codec_address_offset;
			if(codec_address_offset == 0){
				extd_packet->pld.pl0		= packet->pl;
			}
			else{
				extd_packet->pld.pl0		= sbus0->pl;

				extd_packet->pld.pl0.chan14 	= convert_code_to_channel_value(adc[0]);
				extd_packet->pld.pl0.chan15 	= convert_code_to_channel_value(adc[1]);
			}

/*
			extd_packet->pld.pl1		= sbus0->pl;

			extd_packet->pld.pl2.chan0 	= convert_code_to_channel_value(adc[0]);
			extd_packet->pld.pl2.chan1 	= convert_code_to_channel_value(adc[1]);
			extd_packet->pld.pl2.chan2 	= convert_code_to_channel_value(adc[2]);
			extd_packet->pld.pl2.chan3 	= convert_code_to_channel_value(adc[3]);
*/

			extd_packet->crc = crc8_dvb_s2(0, extd_packet->type);					// CRC includes type and payload
			for (int i = 0; i < extd_packet->length - 2; ++i)	extd_packet->crc = crc8_dvb_s2(extd_packet->crc, ((uint8_t*)&(extd_packet->pld))[i]);

			if(++codec_address_offset >= 2) codec_address_offset = 0;
		}
		else if (st == decode){
			if(extd_packet->type == 0x64){
			//CRSF packet
				packet->header = 0xEE;
				packet->length = CRSF_PACKET_LENGTH - 2;
				packet->type = CRSF_PACKET_TYPE;
				packet->pl = extd_packet->pld.pl0;
				packet->crc = crc8_dvb_s2(0, packet->type);					// CRC includes type and payload
				for (int i = 0; i < packet->length - 2; ++i)	packet->crc = crc8_dvb_s2(packet->crc, packet->payload[i]);

			//SBUS packet the same as CRSF payload
				sbus0->header = 0x0F;
				sbus0->b23 = 0x00;
				sbus0->footer = 0x00;
				sbus0->pl	= extd_packet->pld.pl0;


			// ADC & PWM
				TIM3->CCR3 = (uint16_t)((float)convert_channel_value_to_us(extd_packet->pld.pl0.chan14) * TICK_PER_uS);
				TIM3->CCR4 = (uint16_t)((float)convert_channel_value_to_us(extd_packet->pld.pl0.chan15) * TICK_PER_uS);

			// Frequency coding
				f.uFreqCode 		= extd_packet->pld.pl0.chan10;
				f.uFreqCodeRange	= extd_packet->pld.pl0.chan11;
			}
			else{
			//SBUS packet for camera control
				sbus1->header = 0x0F;
				sbus1->b23 = 0x00;
				sbus1->footer = 0x00;
				sbus1->pl	= extd_packet->pld.pl0;

				TIM3->CCR1 = (uint16_t)((float)convert_channel_value_to_us(extd_packet->pld.pl0.chan14)  * TICK_PER_uS);
				TIM3->CCR2 = (uint16_t)((float)convert_channel_value_to_us(extd_packet->pld.pl0.chan15)  * TICK_PER_uS);

			}
/*
			sbus1->pl	= extd_packet->pld.pl1;

			// ADC & PWM

			TIM3->CCR1 = (uint16_t)((float)convert_channel_value_to_us(extd_packet->pld.pl2.chan0)  * TICK_PER_uS);
			TIM3->CCR2 = (uint16_t)((float)convert_channel_value_to_us(extd_packet->pld.pl2.chan1)  * TICK_PER_uS);
*/


		}
		return f;
}
#ifdef TEST_CRSF_TRANSMITTER
void vTestSbusPacketInit(sSbusPacketStr* p_sbus, uint16_t* adc){
	p_sbus->header = 0x0F;
	p_sbus->pl.chan0 = convert_code_to_channel_value(adc[1]);//991;
	p_sbus->pl.chan1 = convert_code_to_channel_value(adc[2]);//992;
	p_sbus->pl.chan2 = 993;
	p_sbus->pl.chan3 = 994;
	p_sbus->pl.chan4 = 995;
	p_sbus->pl.chan5 = 996;
	p_sbus->pl.chan6 = 997;
	p_sbus->pl.chan7 = 998;
	p_sbus->pl.chan8 = 999;
	p_sbus->pl.chan9 = 1000;
	p_sbus->pl.chan10 = 1001;
	p_sbus->pl.chan11 = 1002;
	p_sbus->pl.chan12 = 1003;
	p_sbus->pl.chan13 = 1004;
	p_sbus->pl.chan14 = 1005;
	p_sbus->pl.chan15 = 1006;
	p_sbus->b23 = 0x00;
	p_sbus->footer = 0x00;
}
#endif
