/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"
#include "defines.h"
#include "tm_stm32_fatfs.h"
#include "tm_stm32_adc.h"

/* USER CODE BEGIN defines */
#define KEY_PRESSED     	0x00
#define KEY_NOT_PRESSED 	0x01
#define TORQUE_1        	0xA8
#define TORQUE_3        	0xAA
#define GESCHWINDIGKEIT_RAD	0xCE
#define STAT_KOMBI      	0x1B4
#define GEAR      			0x1D2

#define TORQUE_1_MASK                   0x01
#define TORQUE_3_MASK                   0x02
#define GESCHWINDIGKEIT_RAD_MASK        0x04
#define STAT_KOMBI_MASK                 0x08

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/

volatile uint16_t vehicle_speed;
volatile int16_t trq_eng_in1;  		//      torque actual value can be negative
volatile uint16_t trq_eng_in3;  		//      rpm_engine
volatile int16_t left_wheel_speed; //		velocity of left wheel can be negative

volatile float u1 = 0.0f;
volatile float kp = 10.0f;
float pi = 3.141592f;

volatile int8_t gear;
const float id = 3.15f; 		// ~ 2^4*3.15
const float i1 = 4.7798f; 		// ~ 2^4*4.7798*60/(3.6*2*pi*0.3172)
const float i2 = 3.0562f; 		// ~ 2^4*3.0562*60/(3.6*2*pi*0.3172)
const float i3 = 2.1528f; 		// ~ 2^4*2.1528*60/(3.6*2*pi*0.3172)
const float i4 = 1.6782f; 		// ~ 2^4*1.6782*60/(3.6*2*pi*0.3172)
const float i5 = 1.3901f; 		// ~ 2^4*1.3901*60/(3.6*2*pi*0.3172)
const float i6 = 1.2031f; 		// ~ 2^4*1.2031*60/(3.6*2*pi*0.3172)
const float i7 = 1.0f; 		// ~ 2^4*1*60/(3.6*2*pi*0.3172)
volatile float iges;

static CanTxMsgTypeDef TxMessage;
static CanRxMsgTypeDef RxMessage;
uint8_t ubKeyNumber = 0x0;
uint8_t cantrans_sdstor_init = 0; // cantrans_sdstor_init = 0; Datafile closed, storage has still not begun or has been finished
// cantrans_sdstor_init = 1; Datafile is opened, storage has begun and has not been stopped by the user

uint32_t wbytes;                // Zaehlvariable bezueglich fwrite

uint16_t counter = 0;			// Zaehler fuer CAN Uebertragung
//static uint32_t time_can = 0;

// Variablen zum Ansteuern der Datenspeicherung und Senden der CAN-Signale zur SD-Karte
volatile uint8_t save_files = 0;
// 1.Signal
volatile uint16_t co_arr_save_TR1 = 0;
volatile uint8_t co_arr_full_TR1 = 0;

volatile uint8_t signal_arrvoll_torque1 = 0;

uint8_t button = 0;                                             // 1 pressed
uint32_t FileSize = 0;

/* Fatfs structure */
FATFS FS;
FIL fil_torque1;
FRESULT fres;

/* Size structure for FATFS */
TM_FATFS_Size_t CardSize;

/* Buffer variable */
char buffer[100];
uint8_t arr2SD[2][512];

/* Leonard variablen*/

uint16_t ADC2ConvertedValue = 0;
uint16_t ADC3ConvertedValue = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM2_Init(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC3_Init(void);
static void Error_Handler_CANT(void);
static void Error_Handler_CANR(void);
static void Error_Handler_fats(void);
static void Error_Handler(void);
static void storage_inc_data(volatile uint8_t* save);

void P_regler(void);

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
//
//void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle);

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	MX_DAC_Init();
	MX_ADC3_Init();

	/*##-1- Configure the CAN peripheral #######################################*/
	MX_CAN1_Init();
	MX_CAN2_Init();
	/* Configure TIM2 */
	MX_TIM2_Init();
	/* Configure LED1, LED2, LED3 and LED4 */
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED5);
	BSP_LED_Init(LED6);

	/* Configure Key Button */
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

	// DAC wird gestartet
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

	/*##-2- Start the Reception process and enable reception interrupt #########*/
	if (HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK) {
		/* Reception Error */
		Error_Handler_CANR();
	}
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) { // Initialisierung Timer schon bevor Messung startet
		/* Counter Enable Error */
		Error_Handler();
	}
	if (HAL_ADC_Start_IT(&hadc3) != HAL_OK) {
		/* Counter Enable Error */
		Error_Handler();
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		while (button == 1) {      // Warte bis Knopf gedruckt wird

			if (cantrans_sdstor_init == 0) { // Wurde die Datenuebertragung richtig eingestellt

				cantrans_sdstor_init = 1;           // Setze Initialisierungsbit
				HAL_Delay(1000); // Es wird zur Entprellung des Knopfs benutzt..Nachteilig
				button = 1;

				if (f_mount(&FS, "SD:", 1) != FR_OK) {
					Error_Handler_fats();
				}
				BSP_LED_On(LED5);
			}

			/*##-3- Start the Transmission process ###############################*/
			//Dieses Teil muss geloescht werden, wenn ich am Auto testen will
			if (HAL_CAN_Transmit(&hcan2, 10) != HAL_OK) {
				/* Transmition Error */
				Error_Handler_CANT();

			}
			storage_inc_data(&save_files);
		}
		if (cantrans_sdstor_init == 1 && button == 0) { // wurde Knopf erneut gedruckt bzw. Datenspeicherung gestoppt
			HAL_Delay(1000);
			button = 0;

			/* Unmount SDCARD */
			f_mount(NULL, "SD:", 1);
			BSP_LED_Off(LED5);
			cantrans_sdstor_init = 0; // Die CAN-Uebertragung kann wieder neu gestartet werden
		}
		/* USER CODE END 3 */
	}

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	__PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}

/* CAN1 init function */
void MX_CAN1_Init(void) {
	CAN_FilterConfTypeDef sFilterConfig;

	/*##-1- Configure the CAN peripheral #######################################*/
	hcan1.Instance = CAN1;
	hcan1.pRxMsg = &RxMessage;
	hcan1.Init.Prescaler = 12;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SJW = CAN_SJW_3TQ;
	hcan1.Init.BS1 = CAN_BS1_4TQ;
	hcan1.Init.BS2 = CAN_BS2_2TQ;
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = DISABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = DISABLE;

	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/*##-3- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterNumber = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = TORQUE_1 << 5;
	sFilterConfig.FilterIdLow = TORQUE_3 << 5;
//  sFilterConfig.FilterIdHigh = 0xFF << 5;
//	sFilterConfig.FilterIdLow = 0xFF << 5;
	sFilterConfig.FilterMaskIdHigh = GESCHWINDIGKEIT_RAD << 5; // Nur Radgeschwindigkeit wird aufgenommen
//	sFilterConfig.FilterMaskIdHigh = 0xFF << 5;
//	sFilterConfig.FilterMaskIdLow = STAT_KOMBI << 5;
	sFilterConfig.FilterMaskIdLow = 0xFF << 5;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 0;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	sFilterConfig.FilterNumber = 2;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = GEAR << 5;
	sFilterConfig.FilterIdLow = 0xFF << 5;
	sFilterConfig.FilterMaskIdHigh = 0xFF << 5; // Nur Radgeschwindigkeit wird aufgenommen
	sFilterConfig.FilterMaskIdLow = 0xFF << 5;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 0;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

}

/* CAN2 init function */
void MX_CAN2_Init(void) {

	hcan2.pTxMsg = &TxMessage;
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 12;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SJW = CAN_SJW_3TQ;
	hcan2.Init.BS1 = CAN_BS1_4TQ;
	hcan2.Init.BS2 = CAN_BS2_2TQ;
	hcan2.Init.TTCM = DISABLE;
	hcan2.Init.ABOM = DISABLE;
	hcan2.Init.AWUM = DISABLE;
	hcan2.Init.NART = DISABLE;
	hcan2.Init.RFLM = DISABLE;
	hcan2.Init.TXFP = DISABLE;

	if (HAL_CAN_Init(&hcan2) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/*##-3- Configure Transmission process #####################################*/
//	hcan2.pTxMsg->StdId = STAT_KOMBI;
	hcan2.pTxMsg->StdId = TORQUE_1;
	hcan2.pTxMsg->ExtId = 0x01;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->DLC = 8;

	/* Set the data to be transmitted */
	hcan2.pTxMsg->Data[0] = 72;
	hcan2.pTxMsg->Data[1] = 80;
	hcan2.pTxMsg->Data[2] = 53;
	hcan2.pTxMsg->Data[3] = 61;
	hcan2.pTxMsg->Data[4] = 128;
	hcan2.pTxMsg->Data[5] = 22;
	hcan2.pTxMsg->Data[6] = 99;
	hcan2.pTxMsg->Data[7] = 100;

}

/** Pinout Configuration
 */
void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__GPIOD_CLK_ENABLE()
	;
	__GPIOB_CLK_ENABLE()
	;
	__GPIOC_CLK_ENABLE()
	;

}

/* TIM2 init function */
void MX_TIM2_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 8400;						// Timer laeuft bei 100us
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* DAC init function */
void MX_DAC_Init(void) {

	DAC_ChannelConfTypeDef sConfig;

	/**DAC Initialization
	 */
	hdac.Instance = DAC;
	HAL_DAC_Init(&hdac);

	/**DAC channel OUT1 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
//	HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);
	HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2);
}

/* ADC3 init function */
void MX_ADC3_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	hadc3.Init.Resolution = ADC_RESOLUTION12b;
	hadc3.Init.ScanConvMode = DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc3);

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc3, &sConfig);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void) {
	while (1) {
		BSP_LED_Toggle(LED6);
		HAL_Delay(1000);
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler_CANT(void) {
	f_mount(NULL, "SD:", 1);

	while (1) {
		BSP_LED_Toggle(LED6);
		HAL_Delay(1000);
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler_CANR(void) {
	f_mount(NULL, "SD:", 1);

	while (1) {
		BSP_LED_Toggle(LED6);
		HAL_Delay(1000);
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler_fats(void) {
	f_mount(NULL, "SD:", 1);

	while (1) {
		BSP_LED_Toggle(LED6);
		HAL_Delay(1000);
	}
}

/**
 * @brief  Transmission complete callback in non blocking mode
 * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle) {

	uint8_t help_array = 0;

	/* Receive. This function must be called after each data reception process */
	if (HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0) != HAL_OK) {
		/* Reception Error */
		Error_Handler_CANR();
	}

	switch (CanHandle->pRxMsg->StdId) {
	case (TORQUE_1):     // Drehmoment

		// Drehmoment abspeichern
		if (CanHandle->pRxMsg->Data[2] & 0x80)       // Falls Zahl negativ ist
			trq_eng_in1 = 0xF0 << 8;
		else
			// Falls Zahl positiv ist
			trq_eng_in1 = 0;
		trq_eng_in1 += ((CanHandle->pRxMsg->Data[2]) << 4)
				+ ((CanHandle->pRxMsg->Data[1]) >> 4);
		trq_eng_in1 = trq_eng_in1 >> 1;
		help_array = sprintf(buffer, "%d\n", trq_eng_in1); // Fuelle Array zum ueberpruefen, ob 512 Bytes schon voll sind

		if (512 <= (co_arr_save_TR1 + help_array)) { // Falls zu fuellendes Array keinen Platz mehr hat, dann...
			uint16_t anfang_2fuell = co_arr_save_TR1;
			signal_arrvoll_torque1 = 1;     // Array ist voll

			for (uint8_t i = 0; i < help_array; i++) {

				if (i < 512 - anfang_2fuell) { // Fuelle Array bis 512...
					arr2SD[co_arr_full_TR1][co_arr_save_TR1++] = buffer[i];
				} else {         // den Rest im naechsten Array
					if (co_arr_full_TR1 == 0) { // Falls 1. array voll
						arr2SD[co_arr_full_TR1 + 1][co_arr_save_TR1++] =
								buffer[i]; // ich verwende diese Anweisung co_arr_full_TR1+1, da ich nicht immer wieder co_arr_full_TR1 erhoehen kann
					} else
						// Falls 2. array voll
						arr2SD[co_arr_full_TR1 - 1][co_arr_save_TR1++] =
								buffer[i]; // ich verwende diese Anweisung co_arr_full_TR1+1, da ich nicht immer wieder co_arr_full_TR1 erhoehen kann
				}
				if (co_arr_save_TR1 == 512) // Ist beliebiger Array voll
					co_arr_save_TR1 = 0;    //initialisiere erneut Arrayfuellung
			}

		} else { // Falls zu fuellendes Array noch Platz hat, dann...

			for (uint8_t i = 0; i < help_array; i++)
				arr2SD[co_arr_full_TR1][co_arr_save_TR1++] = buffer[i];

		}

		if (signal_arrvoll_torque1 && ((save_files & TORQUE_1_MASK) == 0)) { // Falls buffer voll und Datenspeicherung gerade nicht stattfindet...
			save_files = save_files | TORQUE_1_MASK; // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
			if (co_arr_full_TR1 == 0) // nachdem am Anfang das 1. array gefuellt wurde
				co_arr_full_TR1++;
			else
				// nachdem das 2. array gefuellt wurde, wird das erste wieder gefuellt
				co_arr_full_TR1--;
		}
//		hcan2.pTxMsg->StdId = TORQUE_3;
		break;

	case (TORQUE_3):     // Speed of the shaft
		trq_eng_in3 = (((CanHandle->pRxMsg->Data[5]) << 8)
				| CanHandle->pRxMsg->Data[4]) >> 2;

//		hcan2.pTxMsg->StdId = GESCHWINDIGKEIT_RAD;
		break;

	case (GESCHWINDIGKEIT_RAD):

		left_wheel_speed = (((CanHandle->pRxMsg->Data[5]) << 8)
				| CanHandle->pRxMsg->Data[4]) >> 4;

//		hcan2.pTxMsg->StdId = GEAR;
		break;

	case (GEAR):

		// Gang abspeichern
		gear = (CanHandle->pRxMsg->Data[1] & 0xF0) >> 4;

		switch (gear) {
		case (5):
			iges = id * i1;
			break;
		case (6):
			iges = id * i2;
			break;
		case (7):
			iges = id * i3;
			break;
		case (8):
			iges = id * i4;
			break;
		case (9):
			iges = id * i5;
			break;
		case (10):
			iges = id * i6;
			break;
		case (11):
			iges = id * i7;
			break;
		default:
			break;

		}

//		hcan2.pTxMsg->StdId = TORQUE_1;
		break;

	default:
		break;
	}

}

/**
 * @brief  EXTI line detection callbacks.
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (button == 0)
		button = 1;
	else
		button = 0;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		// Regler kann hier aufgerufen werden und laeuft mit einer Abtastrate von 100us


	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	if (hadc->Instance == ADC3) {
		ADC3ConvertedValue = HAL_ADC_GetValue(hadc);
		P_regler();
	}

}

static void storage_inc_data(volatile uint8_t *save) { // von hier aus werden die Daten an die SD gesendet, von der isr werden die Daten in die Puffer gespeichert

	if (*save & TORQUE_1_MASK) {

		if ((fres = f_open(&fil_torque1, "SD:torque1.csv",
		FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) == FR_OK) {

			FileSize = f_size(&fil_torque1);

			/* Move to offset of FileSize from top of the file */
			fres = f_lseek(&fil_torque1, FileSize);

			/* Write 512 bytes to file */

			if (co_arr_full_TR1 == 1) { // nachdem am Anfang das 1. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==1
				if (f_write(&fil_torque1, arr2SD[co_arr_full_TR1 - 1], 512,
						(void *) &wbytes) != FR_OK) {
					Error_Handler_fats();
				}
			} else { // nachdem das 2. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==0
				if (f_write(&fil_torque1, arr2SD[co_arr_full_TR1 + 1], 512,
						(void *) &wbytes) != FR_OK) {
					Error_Handler_fats();
				}
			}

			f_close(&fil_torque1);

			// data transfer finished, eventuell muss ich hier die CAN-ISR deaktivieren
			*save = *save & (~TORQUE_1_MASK); // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
			wbytes = 0;
			signal_arrvoll_torque1 = 0;     // Array gesendet
		}
	}

}

/* USER CODE BEGIN 4 */

/**
 *  Ab hier soll Leonards Programm laufen
 */

void P_regler(void) {

	if (cantrans_sdstor_init == 1) { // Regelung wird nach Drucken des Knopfs eingeschaltet
		uint16_t to_dac = 0;

		u1 = ((((float)ADC3ConvertedValue) * 0.1776)-97.2)-(kp*((trq_eng_in3*2*pi/60)-(iges*left_wheel_speed/(3.6*0.3172))));

//		u1 = ((((((int32_t) ADC3ConvertedValue) * 178) - 97200) / 1000))
//				- (((int32_t) (trq_eng_in3
//						- ((iges * (int32_t) left_wheel_speed) >> 8)) << 1) * 10
//						* 314 / 6000);	// 2799 entspricht 2.05 Volt

		if (u1 < 0)
			to_dac = 547;		//entspricht 0.4 V (13.3%) bezueglich 3 V
		else {
			if (u1 > 400)
				to_dac = 2799;		//entspricht 2.05 V bezueglich 3 V
//		to_dac = 4095;		// Bei 4096 laesst er keine Spannung ausgeben
			else
				to_dac = (uint32_t)((u1 + 97.2) / 0.178);
		}

		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, to_dac);
	} else {// Falls Knopf noch nicht gedruckt, dann wird das Hall-Sensorsignal einfach durchgeschaltet
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
				ADC3ConvertedValue);		// ADC durchgeschaltet
	}
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
