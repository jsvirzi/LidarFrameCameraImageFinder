/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */

#include "timegm.h"

enum GprmcFields {
    GprmcMessage = 0,
    GprmcTime,
    GprmcStatus,
    GprmcLatitude,
    GprmcNs,
    GprmcLongitude,
    GprmcEw,
    GprmcKnots,
    GprmcCourse,
    GprmcDate,
    GprmcMagVar,
    GprmcMagEw,
    NumberOfGprmcFields
};

extern uint32_t utcSeconds;

uint32_t UTCTimeFromGPRMCDateTimeStrings(const char *date_string, const char *time_string);
uint32_t UTCTimeFromGPRMCDateTimeStrings(const char *date_string, const char *time_string) {
	struct tm t;
	memset(&t, 0, sizeof(struct tm));

	char str[3];
	str[2] = 0; /* all fields are of length = 2. null terminate */

	str[0] = date_string[0];
	str[1] = date_string[1];
	sscanf(str, "%02d", &t.tm_mday);

	str[0] = date_string[2];
	str[1] = date_string[3];
	sscanf(str, "%02d", &t.tm_mon);
	t.tm_mon--; /* month indices start at 0 */

	str[0] = date_string[4];
	str[1] = date_string[5];
	sscanf(str, "%02d", &t.tm_year);
	t.tm_year += (2000 - 1900); /* 2017 comes as 117 */

	str[0] = time_string[0];
	str[1] = time_string[1];
	sscanf(str, "%02d", &t.tm_hour);

	str[0] = time_string[2];
	str[1] = time_string[3];
	sscanf(str, "%02d", &t.tm_min);

	str[0] = time_string[4];
	str[1] = time_string[5];
	sscanf(str, "%02d", &t.tm_sec);

	const time_t seconds = _timegm_(&t);

	return seconds;
}

extern uint8_t gprmcBuff[];
// extern uint8_t gprmcTemp[];
extern unsigned int gprmcBuffPos;
extern unsigned int gprmcBuffSize;
#define GprmcFields 32
uint8_t *gprmcFields[GprmcFields];
unsigned int gprmcFieldIndex;

#define nLedChannels 20
#define LedChannelMask 31
typedef struct {
	GPIO_TypeDef *gpio;
	uint8_t shiftValue;
} LedChannel;

LedChannel ledChannels[LedChannelMask + 1] = {
	{ GPIOC, 4 },
	{ GPIOC, 5 },
	{ GPIOB, 0 },
	{ GPIOB, 1 },
	{ GPIOB, 2 },
	{ GPIOE, 7 },
	{ GPIOE, 8 },
	{ GPIOE, 9 },
	{ GPIOE, 10 },
	{ GPIOE, 11 },
	{ GPIOE, 12 },
	{ GPIOE, 13 },
	{ GPIOE, 14 },
	{ GPIOE, 15 },
	{ GPIOB, 12 },
	{ GPIOB, 13 },
	{ GPIOB, 14 },
	{ GPIOB, 15 },
	{ GPIOD, 8 },
	{ GPIOD, 9 },
	{ GPIOD, 10 }
};

uint8_t ledChannelPhase = 0;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart6;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

	static ledPhase = 0;
	uint32_t status = htim2.Instance->SR;
	if (status & TIM_SR_CC1IF) {
		ledChannelPhase = 0;
		ledPhase ^= 1;
		GPIOD->BSRR = (1 << (15 + ledPhase * 16));
	}

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

	uint32_t status = htim4.Instance->SR;

	if (status & TIM_SR_UIF) {
		LedChannel *ledChannel = &ledChannels[ledChannelPhase];
		ledChannel->gpio->BSRR = (1 << ledChannel->shiftValue); /* turn on phase */
		uint8_t tempPhase = (ledChannelPhase + nLedChannels - 1) % nLedChannels; /* previous phase */
		ledChannel = &ledChannels[tempPhase];
		ledChannel->gpio->BSRR = (1 << (ledChannel->shiftValue + 16)); /* turn off previous phase */

	}

	if (status & TIM_SR_CC1IF) {
		ledChannelPhase = (ledChannelPhase + 1) % nLedChannels; /* update phase for next interrupt */
		ledChannelPhase &= LedChannelMask; /* unnecessary (for self-healing) */
	}

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles TIM5 global interrupt.
*/
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
* @brief This function handles USART6 global interrupt.
*/
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

	volatile uint8_t ch = huart6.Instance->DR;
	static uint8_t gprmcState = 0;
	uint8_t badState = 0;
	if ((ch == '$') && (gprmcState == 0)) {
		gprmcBuffPos = 0;
		gprmcFieldIndex = 0;
		gprmcState = 1;
		gprmcFields[gprmcFieldIndex] = &gprmcBuff[gprmcBuffPos];
		++gprmcFieldIndex;
	}

//	if (ch != '$') {
//		ch = 'a';
//	}

	if (gprmcState == 1) {
		gprmcBuff[gprmcBuffPos] = ch;
//		gprmcTemp[gprmcBuffPos] = ch;
		if (ch == ',') {
//			gprmcTemp[gprmcBuffPos] = 0;
			gprmcFields[gprmcFieldIndex] = &gprmcBuff[gprmcBuffPos+1];
//			gprmcFields[gprmcFieldIndex] = &gprmcTemp[gprmcBuffPos];
			++gprmcFieldIndex;
			if (gprmcFieldIndex >= GprmcFields) {
				badState = 1;
			}
		}
		++gprmcBuffPos;
		if (gprmcBuffPos >= gprmcBuffSize) {
			badState = 1;
		}
	}

	if ((ch == '*') && (gprmcState == 1)) {
		gprmcBuff[gprmcBuffPos] = ch;
		++gprmcBuffPos;
		if (strncmp(gprmcFields[GprmcMessage], "$GPRMC", 6) == 0) { /* 6 = strlen("$GPRMC") */
			const char *dateString = gprmcFields[GprmcDate];
			const char *timeString = gprmcFields[GprmcTime];
			if (utcSeconds == 0) {
				utcSeconds = UTCTimeFromGPRMCDateTimeStrings(dateString, timeString);
			}
		}
		gprmcState = 0;
	}

	if (badState == 1) {
		gprmcState = 0;
	}

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
