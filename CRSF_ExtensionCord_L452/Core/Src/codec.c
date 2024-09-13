#include "codec.h"
/*
crsf_RC_packed_t_str sVltgToCrsfPacket = {
		.header = 0xEE,
		.length = 0x18, //24d
		.type = 0x16
};
*/

//1400-1600
//800-1100
uint16_t uAdcData[4] = {2048, 2048, 2048, 2048};

static void vSetExtVideoChannel(uint32_t ch){
	if(ch ==0){
		HAL_GPIO_WritePin(VINP_SW_GPIO_Port, VINP_SW_Pin, 1);		// Enable external video input #0
	}
	else if(ch == 1){
		HAL_GPIO_WritePin(VINP_SW_GPIO_Port, VINP_SW_Pin, 0);		// Enable external video input #1
	}
	else return;
}

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

uint16_t convert_us_to_channel_value(uint16_t pwm_us){
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

void vCtrlLinkEncoderRCside(uint16_t* adc,
		sExtendedCrsfPacketStr*	crsf_out,							// Outgoing ExtCRSF packet from Remote control 485 link
		sSbusPacketStr*			sbus_in){							// Incoming SBUS packet from Remote control (Camera control)
			crsf_out->header		= 0xEE;
			crsf_out->length		= EXT_CTRL_LINK_PACKET_LENGTH - 2;
			crsf_out->type			= EXT_CTRL_LINK_PACKET_TYPE;
			crsf_out->pld.pl0		= sbus_in->pl;

			crsf_out->pld.pl1.chan0  	= convert_code_to_channel_value(adc[0]);
			crsf_out->pld.pl1.chan1 	= convert_code_to_channel_value(adc[1]);
			crsf_out->pld.pl1.chan2  	= convert_code_to_channel_value(adc[2]);
			crsf_out->pld.pl1.chan3 	= convert_code_to_channel_value(adc[3]);
			crsf_out->crc = crc8_dvb_s2(0, crsf_out->type);	// ExtCRC includes type and payload
			for (int i = 0; i < crsf_out->length - 2; ++i)	crsf_out->crc = crc8_dvb_s2(crsf_out->crc, ((uint8_t*)&(crsf_out->pld))[i]);
}

void vCtrlLinkDecoderRFside(
		sExtendedCrsfPacketStr* extd_packet,			// Extended CRSF packet is used for communication between the uCUs on both sides
		sSbusPacketStr* sbus_out						// Preparing SBUS packet used for Camera control on RF side
		){
			//SBUS packet for camera control
			sbus_out->header = 0x0F;
			sbus_out->b23 = 0x00;
			sbus_out->footer = 0x00;
			sbus_out->pl	= extd_packet->pld.pl0;

			// ADC to PWM
			TIM3->CCR1 = (uint16_t)((float)convert_channel_value_to_us(extd_packet->pld.pl1.chan0)  * TICK_PER_uS);//adc[0]
			TIM3->CCR2 = (uint16_t)((float)convert_channel_value_to_us(extd_packet->pld.pl1.chan1)  * TICK_PER_uS);//adc[1]
}

sFreqCodeStr vCrsfDecoderRFside(
		sCrsfPacketStr* crsf,							// Incoming CRSF packet from Remote control
		sSbusPacketStr* sbus){							// Outgoing SBUS packet with the same payload as in incoming CRSF
	static sFreqCodeStr f = {1000, 1000};
	//SBUS packet the same as CRSF payload
		sbus->header = 0x0F;
		sbus->b23 = 0x00;
		sbus->footer = 0x00;
		sbus->pl	= crsf->pl;


	// External video switch control
		TIM15->CCR1 = (uint16_t)((float)convert_channel_value_to_us(crsf->pl.chan14) * TICK_PER_uS);
		TIM15->CCR2 = (uint16_t)((float)convert_channel_value_to_us(crsf->pl.chan15) * TICK_PER_uS);
		if((convert_channel_value_to_us(crsf->pl.chan15) >= 800)
				&& (convert_channel_value_to_us(crsf->pl.chan15) <= 1100)){
			vSetExtVideoChannel(0);
		}
		else if((convert_channel_value_to_us(crsf->pl.chan15) >= 1400)
				&& (convert_channel_value_to_us(crsf->pl.chan15) <= 1600)){
			vSetExtVideoChannel(1);
		}
		else
			vSetExtVideoChannel(0);


	// Frequency coding
		f.uFreqCode 		= crsf->pl.chan10;
		f.uFreqCodeRange	= crsf->pl.chan11;

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
