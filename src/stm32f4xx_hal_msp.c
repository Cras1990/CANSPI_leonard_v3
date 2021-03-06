/**
 ******************************************************************************
 * File Name          : stm32f4xx_hal_msp.c
 * Description        : This file provides code for the MSP Initialization
 *                      and de-Initialization codes.
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

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
	/* USER CODE BEGIN MspInit 0 */

	/* USER CODE END MspInit 0 */

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* System interrupt init*/
	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

	/* USER CODE BEGIN MspInit 1 */

	/* USER CODE END MspInit 1 */
}

static int CAN1_CLK_ENABLED = 0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hcan->Instance == CAN1) {
		/* USER CODE BEGIN CAN1_MspInit 0 */

		/* USER CODE END CAN1_MspInit 0 */
		/* Peripheral clock enable */
		CAN1_CLK_ENABLED++;
		if (CAN1_CLK_ENABLED == 1) {
			__CAN1_CLK_ENABLE()
			;
		}

		/**CAN1 GPIO Configuration
		 PD0     ------> CAN1_RX
		 PD1     ------> CAN1_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(CAN1_TX_IRQn, 7, 0);
		HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
//    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		/* USER CODE BEGIN CAN1_MspInit 1 */

		/* USER CODE END CAN1_MspInit 1 */
	} else if (hcan->Instance == CAN2) {
		/* USER CODE BEGIN CAN2_MspInit 0 */

		/* USER CODE END CAN2_MspInit 0 */
		/* Peripheral clock enable */
		__CAN2_CLK_ENABLE()
		;
		CAN1_CLK_ENABLED++;
		if (CAN1_CLK_ENABLED == 1) {
			__CAN1_CLK_ENABLE()
			;
		}

		/**CAN2 GPIO Configuration
		 PB5     ------> CAN2_RX
		 PB6     ------> CAN2_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(CAN2_TX_IRQn, 8, 0);
		HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
		HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 8, 0);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		/* USER CODE BEGIN CAN2_MspInit 1 */

		/* USER CODE END CAN2_MspInit 1 */
	}

}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan) {

	if (hcan->Instance == CAN1) {
		/* USER CODE BEGIN CAN1_MspDeInit 0 */

		/* USER CODE END CAN1_MspDeInit 0 */
		/* Peripheral clock disable */
		CAN1_CLK_ENABLED--;
		if (CAN1_CLK_ENABLED == 0) {
			__CAN1_CLK_DISABLE();
		}

		/**CAN1 GPIO Configuration
		 PD0     ------> CAN1_RX
		 PD1     ------> CAN1_TX
		 */
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);

		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);

		/* USER CODE BEGIN CAN1_MspDeInit 1 */

		/* USER CODE END CAN1_MspDeInit 1 */
	} else if (hcan->Instance == CAN2) {
		/* USER CODE BEGIN CAN2_MspDeInit 0 */

		/* USER CODE END CAN2_MspDeInit 0 */
		/* Peripheral clock disable */
		__CAN2_CLK_DISABLE();
		CAN1_CLK_ENABLED--;
		if (CAN1_CLK_ENABLED == 0) {
			__CAN1_CLK_DISABLE();
		}

		/**CAN2 GPIO Configuration
		 PB5     ------> CAN2_RX
		 PB6     ------> CAN2_TX
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);

		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);

		/* USER CODE BEGIN CAN2_MspDeInit 1 */

		/* USER CODE END CAN2_MspDeInit 1 */
	}

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {

	if (htim_base->Instance == TIM2) {
		/* USER CODE BEGIN TIM2_MspInit 0 */

		/* USER CODE END TIM2_MspInit 0 */
		/* Peripheral clock enable */
		__TIM2_CLK_ENABLE()
		;
		/* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
		/* USER CODE BEGIN TIM2_MspInit 1 */

		/* USER CODE END TIM2_MspInit 1 */
	}

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {

	if (htim_base->Instance == TIM2) {
		/* USER CODE BEGIN TIM2_MspDeInit 0 */

		/* USER CODE END TIM2_MspDeInit 0 */
		/* Peripheral clock disable */
		__TIM2_CLK_DISABLE();

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(TIM2_IRQn);

	}
	/* USER CODE BEGIN TIM2_MspDeInit 1 */

	/* USER CODE END TIM2_MspDeInit 1 */

}

/* USER CODE BEGIN 1 */

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hadc->Instance == ADC3) {
		/* USER CODE BEGIN ADC3_MspInit 0 */

		/* USER CODE END ADC3_MspInit 0 */
		/* Peripheral clock enable */
		__ADC3_CLK_ENABLE()
		;

		/**ADC3 GPIO Configuration
		 PC1     ------> ADC3_IN11
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/* Peripheral interrupt init*/
		HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(ADC_IRQn);

		/* USER CODE BEGIN ADC3_MspInit 1 */

		/* USER CODE END ADC3_MspInit 1 */
	}

}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {

	if (hadc->Instance == ADC3) {
		/* USER CODE BEGIN ADC3_MspDeInit 0 */

		/* USER CODE END ADC3_MspDeInit 0 */
		/* Peripheral clock disable */
		__ADC3_CLK_DISABLE();

		/**ADC3 GPIO Configuration
		 PC2     ------> ADC3_IN12
		 */
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1);

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(ADC_IRQn);

	}

	/* USER CODE END ADC3_MspDeInit 1 */

}

void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hdac->Instance == DAC) {
		/* USER CODE BEGIN DAC_MspInit 0 */

		/* USER CODE END DAC_MspInit 0 */
		/* Peripheral clock enable */
		__DAC_CLK_ENABLE()
		;

		/**DAC GPIO Configuration
		 PA4     ------> DAC_OUT1
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE BEGIN DAC_MspInit 1 */

		/**DAC GPIO Configuration
		 PA5     ------> DAC_OUT2
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE END DAC_MspInit 1 */
	}

}

void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac) {

	if (hdac->Instance == DAC) {
		/* USER CODE BEGIN DAC_MspDeInit 0 */

		/* USER CODE END DAC_MspDeInit 0 */
		/* Peripheral clock disable */
		__DAC_CLK_DISABLE();

		/**DAC GPIO Configuration
		 PA4     ------> DAC_OUT1
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
	}
	/* USER CODE BEGIN DAC_MspDeInit 1 */

	/* USER CODE END DAC_MspDeInit 1 */

}

/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
