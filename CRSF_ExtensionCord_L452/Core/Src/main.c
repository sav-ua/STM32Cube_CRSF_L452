/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "codec.h"
#include "camera.h"
#include "filter.h"
#include <string.h>
//#include "usbd_cdc_if.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define X_RANGE_ENABLED

//CRSF_FRAMETYPE_COMMAND
#define CRSF_FRAMETYPE_LEN 10
const uint8_t uCrsfFrameCommand[] = {0xC8, 0x08, 0x32, 0xEE, 0xEA, 0x10, 0x05, 0x03, 0xA8, 0xC5};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_lpuart_tx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
#ifdef TEST_CRSF_TRANSMITTER
static uint8_t		uSbusTxBuffer[SBUS_PACKET_LENGTH];
const uint8_t uTxBuffer[3][CRSF_PACKET_LENGTH] = {
	{0xEE,0x18,0x16,0x12,0xC1,0xCB,0x77,0x8A,0xC4,0x2A,0x89,0xE5,0xED,0x7B,0x45,0x64,0xA5,0x44,0xF3,0xFA,0x5D,0x23,0xB3,0x7A,0xE2,0x55},
	{0xEE,0x18,0x16,0x13,0x77,0x1F,0xF9,0xB8,0x37,0xF1,0x89,0x4F,0xBC,0x15,0xE0,0x13,0x1E,0xF8,0xC0,0x07,0x3E,0xF0,0x81,0x0F,0x7C,0xDF},
	{0xEE,0x18,0x16,0x40,0x74,0x1F,0xF9,0xB6,0x37,0xF1,0x89,0x4F,0xBC,0x15,0xE0,0x1B,0x1E,0xF8,0xC0,0x07,0x3E,0xF0,0x81,0x0F,0x7C,0xCD}
};
#else
static uint8_t		uTxBuffer[CRSF_PACKET_LENGTH];
#endif
static uint8_t		uExtendedTxBuffer[EXTENDED_CRSF_PACKET_LENGTH];
static uint8_t		uRxBuffer[200];

static uint8_t		uSbusRxBuffer[SBUS_PACKET_LENGTH + 1];

uint16_t	uAdcBuf[4];


static sSbusPacketStr sCameraCtrlSbusEncoder;

static sCrsfPacketStr sCrsfPacketDecoder;
static sSbusPacketStr sCameraCtrlSbusPacketDecoder;
static sSbusPacketStr sCrsfToSbusPacketDecoder;


// Radio receiver frequency coding (PA9 USART1_TX)
#define RADIO_CODE_CH_CNT	8

volatile sFreqCodeStr sFreqCode = {1000, 1000};


typedef struct{
	uint32_t	low_frq;
	uint32_t	hi_frq;
	uint8_t		frq_code[6];
}sRadioCodeStr;

typedef struct{
	uint8_t			uRadioSincro[6];
	sRadioCodeStr	sRadioCodeR[RADIO_CODE_CH_CNT];
	sRadioCodeStr	sRadioCodeL[RADIO_CODE_CH_CNT];
#ifdef X_RANGE_ENABLED
	sRadioCodeStr	sRadioCodeX[RADIO_CODE_CH_CNT];
#endif
	uint32_t		uRangeR[2];
	uint32_t		uRangeL[2];
#ifdef X_RANGE_ENABLED
	uint32_t		uRangeX[2];
#endif
}sRadioFreqStr;

const sRadioFreqStr sRadioFreq = {
		.uRadioSincro = {0x02, 0x06, 0x33, 0x80, 0xB5, 0x03},
		.sRadioCodeR = {

				{172 , 420 , {0x02,	0x06,	0x31,	0x20,	0x17,	0x03}},
				{426 , 649 , {0x02,	0x06,	0x31,	0x21,	0x16,	0x03}},
				{655 , 879 , {0x02,	0x06,	0x31,	0x22,	0x15,	0x03}},
				{886 , 1107, {0x02,	0x06,	0x31,	0x23,	0x14,	0x03}},
				{1115, 1337, {0x02,	0x06,	0x31,	0x24,	0x13,	0x03}},
				{1344, 1566, {0x02,	0x06,	0x31,	0x25,	0x12,	0x03}},
				{1573, 1787, {0x02,	0x06,	0x31,	0x26,	0x11,	0x03}},
				{1793, 1811, {0x02,	0x06,	0x31,	0x27,	0x10,	0x03}}
		},
#ifdef X_RANGE_ENABLED
		.sRadioCodeX = {
				{172 , 420 , {0x02, 0x06, 0x31, 0x30, 0x07, 0x03}},
				{426 , 649 , {0x02, 0x06, 0x31, 0x31, 0x06, 0x03}},
				{655 , 879 , {0x02, 0x06, 0x31, 0x32, 0x05, 0x03}},
				{886 , 1107, {0x02, 0x06, 0x31, 0x33, 0x04, 0x03}},
				{1115, 1337, {0x02, 0x06, 0x31, 0x34, 0x03, 0x03}},
				{1344, 1566, {0x02, 0x06, 0x31, 0x35, 0x02, 0x03}},
				{1573, 1787, {0x02, 0x06, 0x31, 0x36, 0x01, 0x03}},
				{1793, 1811, {0x02, 0x06, 0x31, 0x37, 0x00, 0x03}}

		},
#endif
		.sRadioCodeL = {
				{172 , 420 , {0x02,	0x06,	0x31,	0x28,	0x1F,	0x03}},
				{426 , 649 , {0x02,	0x06,	0x31,	0x29,	0x1E,	0x03}},
				{655 , 879 , {0x02,	0x06,	0x31,	0x2A,	0x1D,	0x03}},
				{886 , 1107, {0x02,	0x06,	0x31,	0x2B,	0x1C,	0x03}},
				{1115, 1337, {0x02,	0x06,	0x31,	0x2C,	0x1B,	0x03}},
				{1344, 1566, {0x02,	0x06,	0x31,	0x2D,	0x1A,	0x03}},
				{1573, 1787, {0x02,	0x06,	0x31,	0x2E,	0x19,	0x03}},
				{1793, 1811, {0x02,	0x06,	0x31,	0x2F,	0x18,	0x03}}
		},
		.uRangeR = {911, 1072},
		.uRangeL = {172, 831},
#ifdef X_RANGE_ENABLED
		.uRangeX = {1152, 1811}
#endif

};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define ADC_COMPRESS_RATIO_PCT 25.0f


#define ADC_MIN_US 0.0f
#define ADC_MAX_US 4095.0f

#define ADC_MIN_CMPR_US (ADC_MIN_US + (ADC_MAX_US - ADC_MIN_US) / 100 * ADC_COMPRESS_RATIO_PCT)
#define ADC_MAX_CMPR_US (ADC_MAX_US - (ADC_MAX_US - ADC_MIN_US) / 100 * ADC_COMPRESS_RATIO_PCT)
#define ADC_SCF 		((ADC_MAX_US - ADC_MIN_US)/(ADC_MAX_CMPR_US - ADC_MIN_CMPR_US))


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	uint16_t uAdcTemp[4];
	for(int i = 0; i < 4; i++) uAdcTemp[i] = (uint16_t)uAdcBuf[i];
//------------------------ ADC ch0 Input range compressing --------------------------------------
	if(uAdcTemp[0] <= ADC_MIN_CMPR_US) uAdcTemp[0] = ADC_MIN_CMPR_US;
	if(uAdcTemp[0] >= ADC_MAX_CMPR_US) uAdcTemp[0] = ADC_MAX_CMPR_US;
	uAdcTemp[0] = (uint16_t)(((float)uAdcTemp[0] - ADC_MIN_CMPR_US) * ADC_SCF);
	memcpy((uint8_t*)uAdcData,(uint8_t*)uAdcTemp, sizeof(uAdcTemp));
//----------------------------------------------------------------------------------------------
	movingAvgFlt(uAdcData);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	uint8_t		crc = 0;
	if(huart == &UART_SBUS){
		if( (((sSbusPacketStr*)uSbusRxBuffer)->header == 0x0F) && (((sSbusPacketStr*)uSbusRxBuffer)->footer == 0x00) ){
			memcpy((uint8_t*)&sCameraCtrlSbusEncoder, uSbusRxBuffer, sizeof(sCameraCtrlSbusEncoder));
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&UART_SBUS, (uint8_t*)uSbusRxBuffer, sizeof(uSbusRxBuffer));
	}
	else if(huart == &UART_CRSF){
		if(((sCrsfPacketStr*)uRxBuffer)->type ==  CRSF_PACKET_TYPE){ // CRSF packet.CODER side
			crc = crc8_dvb_s2(0, ((sCrsfPacketStr*)uRxBuffer)->type);	// CRC includes type and payload
			for (int i = 0; i < CRSF_PAYLOAD_LENGTH; ++i)	crc = crc8_dvb_s2(crc, ((sCrsfPacketStr*)uRxBuffer)->payload[i]);
			if(crc == ((sCrsfPacketStr*)uRxBuffer)->crc){
				memcpy(uTxBuffer, uRxBuffer, sizeof(uTxBuffer));
				vExtendedPacketCoding(uAdcData,
						(sCrsfPacketStr*)uTxBuffer,
						(sExtendedCrsfPacketStr*)uExtendedTxBuffer,
						&sCameraCtrlSbusEncoder,
						(void*)0/*&sCameraCtrlSbusPacket*/,
						encoder);
				HAL_UART_Transmit_DMA(&UART_CRSF, uExtendedTxBuffer, sizeof(sExtendedCrsfPacketStr));
			}
		}
		else if((((sExtendedCrsfPacketStr*)uRxBuffer)->type & EXTENDED_CRSF_PACKET_TYPE) == EXTENDED_CRSF_PACKET_TYPE){ // extended CRSF packet. DECODER side
			crc = crc8_dvb_s2(0, ((sExtendedCrsfPacketStr*)uRxBuffer)->type);	// CRC includes type and payload
			for (int i = 0; i < CRSF_PAYLOAD_LENGTH * EXTENDED_PAYLOAD_CNT; ++i)	crc = crc8_dvb_s2(crc, ((uint8_t*)&(((sExtendedCrsfPacketStr*)uRxBuffer)->pld))[i]);
			if(crc == ((sExtendedCrsfPacketStr*)uRxBuffer)->crc){
				memcpy(uExtendedTxBuffer, uRxBuffer, sizeof(uExtendedTxBuffer));

				sFreqCode = vExtendedPacketCoding(uAdcData,
						&sCrsfPacketDecoder/*(sCrsfPacketStr*)uTxBuffer*/,
						(sExtendedCrsfPacketStr*)uExtendedTxBuffer,
						&sCrsfToSbusPacketDecoder,
						&sCameraCtrlSbusPacketDecoder,
						decode);
				HAL_UART_Transmit_DMA(&UART_CRSF, (uint8_t*)&sCrsfPacketDecoder/*uTxBuffer*/, sizeof(sCrsfPacketStr));
				HAL_UART_Transmit_DMA(&UART_SBUS, (uint8_t*)&sCrsfToSbusPacketDecoder, sizeof(sSbusPacketStr));
			}
		}
		else{
			if(Size <= sizeof(uRxBuffer)){ //Any other packets with length less than 200 bytes will be transmitted without any changes
				HAL_UART_Transmit_DMA(&UART_CRSF, uRxBuffer, Size);
			}
		}
		//memset(uRxBuffer, 0, sizeof(uRxBuffer));
		HAL_UARTEx_ReceiveToIdle_DMA(&UART_CRSF, (uint8_t*)uRxBuffer, sizeof(uRxBuffer));
	}
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle){
	if(UartHandle == &UART_CRSF){
		__HAL_UART_CLEAR_PEFLAG(&UART_CRSF);
		__HAL_UART_CLEAR_OREFLAG(&UART_CRSF);
		__HAL_UART_CLEAR_FEFLAG(&UART_CRSF);
		__HAL_UART_CLEAR_NEFLAG(&UART_CRSF);
		__HAL_UART_CLEAR_IDLEFLAG(&UART_CRSF);
		HAL_UARTEx_ReceiveToIdle_DMA(&UART_CRSF, (uint8_t*)uRxBuffer, sizeof(uRxBuffer));
	}
	else if(UartHandle == &UART_SBUS){
		__HAL_UART_CLEAR_PEFLAG(&UART_SBUS);
		__HAL_UART_CLEAR_OREFLAG(&UART_SBUS);
		__HAL_UART_CLEAR_FEFLAG(&UART_SBUS);
		__HAL_UART_CLEAR_NEFLAG(&UART_SBUS);
		__HAL_UART_CLEAR_IDLEFLAG(&UART_SBUS);
		HAL_UARTEx_ReceiveToIdle_DMA(&UART_SBUS, (uint8_t*)uSbusRxBuffer, sizeof(uSbusRxBuffer));
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	static volatile uint32_t delay_cycle = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_USB_PCD_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);



#ifdef TEST_CRSF_TRANSMITTER
	HAL_ADC_Start_DMA (&hadc1, uAdcBuf, 4);
	while (1){
		static uint32_t c = 0;
		HAL_UART_Transmit(&UART_CRSF, (uint8_t*)&uTxBuffer[c][0], CRSF_PACKET_LENGTH, 5);
		if(c++ >= 2) c = 0;

		HAL_Delay(3);
		HAL_UART_Transmit(&UART_CRSF, uCrsfFrameCommand, sizeof(uCrsfFrameCommand), 5);
		HAL_Delay(1);

		vTestSbusPacketInit((sSbusPacketStr*)uSbusTxBuffer, uAdcData);
		HAL_UART_Transmit_DMA(&UART_SBUS, uSbusTxBuffer, sizeof(sSbusPacketStr));
	}
#else
	HAL_UARTEx_ReceiveToIdle_DMA(&UART_CRSF, (uint8_t*)uRxBuffer, sizeof(uRxBuffer));
//---- SBUS data receiving ----
	HAL_UARTEx_ReceiveToIdle_DMA(&UART_SBUS, (uint8_t*)uSbusRxBuffer, sizeof(uSbusRxBuffer));
//-----------------------------
	HAL_ADC_Start_DMA (&hadc1, (uint32_t*)uAdcBuf, 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GPIO_ReadPin(MVIDEO_GPIO_Port, MVIDEO_Pin)){ // RF side. RLED- ON
		  HAL_GPIO_WritePin(RLED_GPIO_Port, RLED_Pin, 0);
	  }
	  else{													// Remote control side. GLED - OFF
		  HAL_GPIO_WritePin(RLED_GPIO_Port, RLED_Pin, 1);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//------------------ Enable Alternative USART1 output USART1_TX = PB6 ---------------------------------------------------------
//		vUSART1_PinRemap(usart1_tx_pb6);
		vConvertSbusToCameraProtocol(&sCameraCtrlSbusPacketDecoder);
//		vUSART1_PinRemap(usart1_tx_pa9);
//------------------ Disable Alternative USART1 output USART1_TX = PA9 ---------------------------------------------------------
		if(delay_cycle == 50){	// 0.5s
			HAL_UART_Transmit_DMA(&UART_FRQ_CTRL, sRadioFreq.uRadioSincro, sizeof(sRadioFreq.uRadioSincro));
		}
		if(delay_cycle == 100){ // 1s
			volatile uint32_t uFreqCodeCopy;
			volatile uint32_t uFreqCodeRangeCopy;
				uFreqCodeCopy		= sFreqCode.uFreqCode;
				uFreqCodeRangeCopy	= sFreqCode.uFreqCodeRange;
				if((uFreqCodeRangeCopy >= sRadioFreq.uRangeR[0]) && (uFreqCodeRangeCopy <= sRadioFreq.uRangeR[1])){ // R - range
					for(int i = 0; i < RADIO_CODE_CH_CNT; i++){
						if((uFreqCodeCopy >= sRadioFreq.sRadioCodeR[i].low_frq) && (uFreqCodeCopy <= sRadioFreq.sRadioCodeR[i].hi_frq)){
							HAL_UART_Transmit_DMA(&UART_FRQ_CTRL, sRadioFreq.sRadioCodeR[i].frq_code, sizeof(sRadioFreq.sRadioCodeR[i].frq_code));
						}
					}
				}
				else if((uFreqCodeRangeCopy >= sRadioFreq.uRangeL[0]) && (uFreqCodeRangeCopy <= sRadioFreq.uRangeL[1])){ // L - range
					for(int i = 0; i < RADIO_CODE_CH_CNT; i++){
						if((uFreqCodeCopy >= sRadioFreq.sRadioCodeL[i].low_frq) && (uFreqCodeCopy <= sRadioFreq.sRadioCodeL[i].hi_frq)){
							HAL_UART_Transmit_DMA(&UART_FRQ_CTRL, sRadioFreq.sRadioCodeL[i].frq_code, sizeof(sRadioFreq.sRadioCodeL[i].frq_code));
						}
					}
				}
#ifdef X_RANGE_ENABLED
				else if((uFreqCodeRangeCopy >= sRadioFreq.uRangeX[0]) && (uFreqCodeRangeCopy <= sRadioFreq.uRangeX[1])){ // X - range
					for(int i = 0; i < RADIO_CODE_CH_CNT; i++){
						if((uFreqCodeCopy >= sRadioFreq.sRadioCodeX[i].low_frq) && (uFreqCodeCopy <= sRadioFreq.sRadioCodeX[i].hi_frq)){
							HAL_UART_Transmit_DMA(&UART_FRQ_CTRL, sRadioFreq.sRadioCodeX[i].frq_code, sizeof(sRadioFreq.sRadioCodeX[i].frq_code));
						}
					}
				}
#endif


		}
		if(++delay_cycle > 100)	delay_cycle = 0;
		HAL_Delay(10 - 3);// 3 - correction. The required latency should be 10ms.
  }
#endif
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 400000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT;
  huart2.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 20000;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 31;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 50000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 30000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 40000;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RLED_GPIO_Port, RLED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : RLED_Pin */
  GPIO_InitStruct.Pin = RLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MVIDEO_Pin */
  GPIO_InitStruct.Pin = MVIDEO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MVIDEO_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
