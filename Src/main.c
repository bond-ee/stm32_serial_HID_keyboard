/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "hid_keyboard.h"

//#define OSX

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
	struct keyboardHID_t {
		uint8_t modifiers;
		uint8_t reserved;
		uint8_t key1;
	};
	struct keyboardHID_t keyboardHID;
	keyboardHID.modifiers = 0;
	keyboardHID.key1 = 0;

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	MX_USB_DEVICE_Init();

	/* USER CODE BEGIN 2 */
	uint8_t receivedData[3];
	uint8_t send = 0;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		receivedData[0] = 0;
		receivedData[1] = 0;
		receivedData[2] = 0;

		HAL_UART_Receive(&huart3, receivedData, 3, 10);
		HAL_UART_Transmit(&huart3, receivedData, 3, 10);

		//Capital letters
		if (receivedData[0] >= 0x41 && receivedData[0] <= 0x5A) {
			keyboardHID.key1 = KB_A + (receivedData[0] - 0x41);
			keyboardHID.modifiers = 0x02;
			send = 1;
		}
		//lower case letters
		else if (receivedData[0] >= 0x61 && receivedData[0] <= 0x7A) {
			keyboardHID.key1 = KB_A + (receivedData[0] - 0x61);
			keyboardHID.modifiers = 0;
			send = 1;
		}
		//Enter
		else if (receivedData[0] == 0x0D) {
			keyboardHID.key1 = KB_ENTER;
			keyboardHID.modifiers = 0;
			send = 1;
		}
		//Space
		else if (receivedData[0] == 0x20) {
			keyboardHID.key1 = KB_SPACEBAR;
			keyboardHID.modifiers = 0;
			send = 1;
		}
		//Numbers 1-9
		else if (receivedData[0] >= 0x31 && receivedData[0] <= 0x39) {
			keyboardHID.key1 = KB_1 + (receivedData[0] - 0x31);
			keyboardHID.modifiers = 0;
			send = 1;
		}
		//Number 0
		else if (receivedData[0] == 0x30) {
			keyboardHID.key1 = KB_0;
			keyboardHID.modifiers = 0;
			send = 1;
		}
		//!
		else if (receivedData[0] == ASCII_EXCLAMATION) {
			keyboardHID.key1 = KB_1;
			keyboardHID.modifiers = 0x02;
			send = 1;
		}
		//@
		else if (receivedData[0] == ASCII_AT) {
			keyboardHID.key1 = KB_2;
			keyboardHID.modifiers = 0x02;
			send = 1;
		}
		//#
		else if (receivedData[0] == ASCII_POUND) {
			keyboardHID.key1 = KB_3;
			keyboardHID.modifiers = 0x02;
			send = 1;
		}
		//$
		else if (receivedData[0] == ASCII_DOLLAR) {
			keyboardHID.key1 = KB_4;
			keyboardHID.modifiers = 0x02;
			send = 1;
		}

		//%
		else if (receivedData[0] == ASCII_PERCENTAGE) {
			keyboardHID.key1 = KB_5;
			keyboardHID.modifiers = 0x02;
			send = 1;
		}
		//^
		else if (receivedData[0] == ASCII_CARET) {
			keyboardHID.key1 = KB_6;
			keyboardHID.modifiers = 0x02;
			send = 1;
		}
		//&
		else if (receivedData[0] == ASCII_AND) {
			keyboardHID.key1 = KB_7;
			keyboardHID.modifiers = 0x02;
			send = 1;
		}
		//*
		else if (receivedData[0] == ASCII_STAR) {
			keyboardHID.key1 = KB_8;
			keyboardHID.modifiers = 0x02;
			send = 1;
		}
		//(
		else if (receivedData[0] == ASCII_LEFT_PARENTH) {
			keyboardHID.key1 = KB_9;
			keyboardHID.modifiers = 0x02;
			send = 1;
		}
		//)
		else if (receivedData[0] == ASCII_RIGHT_PARENTH) {
			keyboardHID.key1 = KB_0;
			keyboardHID.modifiers = 0x02;
			send = 1;
		}

		//`
		else if (receivedData[0] == ASCII_ACUTE) {
					keyboardHID.key1 = KB_ACUTE;
					keyboardHID.modifiers = 0x00;
					send = 1;
				}
		//~
		else if (receivedData[0] == ASCII_TILDE) {
					keyboardHID.key1 = KB_ACUTE;
					keyboardHID.modifiers = 0x02;
					send = 1;
				}
		//{
		else if (receivedData[0] == ASCII_OPEN_BRACE) {
					keyboardHID.key1 = KB_OPEN_BRACKET;
					keyboardHID.modifiers = 0x02;
					send = 1;
				}
		//[
		else if (receivedData[0] == ASCII_OPEN_BRACKET) {
					keyboardHID.key1 = KB_OPEN_BRACKET;
					keyboardHID.modifiers = 0x00;
					send = 1;
				}
		//}
		else if (receivedData[0] == ASCII_CLOSE_BRACE) {
					keyboardHID.key1 = KB_CLOSE_BRACKET;
					keyboardHID.modifiers = 0x02;
					send = 1;
				}
		//]
		else if (receivedData[0] == ASCII_CLOSE_BRACKET) {
					keyboardHID.key1 = KB_CLOSE_BRACKET;
					keyboardHID.modifiers = 0x00;
					send = 1;
				}
		// \

		else if (receivedData[0] == ASCII_BACKSLASH) {
					keyboardHID.key1 = KB_BACKSLASH;
					keyboardHID.modifiers = 0x00;
					send = 1;
				}
		//|
		else if (receivedData[0] == ASCII_PIPE) {
					keyboardHID.key1 = KB_BACKSLASH;
					keyboardHID.modifiers = 0x02;
					send = 1;
				}
		//:
		else if (receivedData[0] == ASCII_COLON) {
					keyboardHID.key1 = KB_SEMICOLON;
					keyboardHID.modifiers = 0x02;
					send = 1;
				}
		//;
		else if (receivedData[0] == ASCII_SEMICOLON) {
					keyboardHID.key1 = KB_SEMICOLON;
					keyboardHID.modifiers = 0x00;
					send = 1;
				}
		//"
		else if (receivedData[0] == ASCII_QUOTE) {
					keyboardHID.key1 = KB_QUOTE;
					keyboardHID.modifiers = 0x02;
					send = 1;
				}
		//'
		else if (receivedData[0] == ASCII_APOSTROPHE) {
					keyboardHID.key1 = KB_QUOTE;
					keyboardHID.modifiers = 0x00;
					send = 1;
				}
		//<
		else if (receivedData[0] == ASCII_LESS_THAN) {
					keyboardHID.key1 = KB_LESS_THAN;
					keyboardHID.modifiers = 0x02;
					send = 1;
				}
		//,
		else if (receivedData[0] == ASCII_COMMA) {
					keyboardHID.key1 = KB_LESS_THAN;
					keyboardHID.modifiers = 0x00;
					send = 1;
				}
		//>
		else if (receivedData[0] == ASCII_GREATER_THAN) {
					keyboardHID.key1 = KB_GREATER_THAN;
					keyboardHID.modifiers = 0x02;
					send = 1;
				}
		//.
		else if (receivedData[0] == ASCII_PERIOD) {
					keyboardHID.key1 = KB_GREATER_THAN;
					keyboardHID.modifiers = 0x00;
					send = 1;
				}
		//?
		else if (receivedData[0] == ASCII_QUESTION_MARK) {
					keyboardHID.key1 = KB_FORWARD_SLASH;
					keyboardHID.modifiers = 0x02;
					send = 1;
				}
		///
		else if (receivedData[0] == ASCII_FORWARD_SLASH) {
					keyboardHID.key1 = KB_FORWARD_SLASH;
					keyboardHID.modifiers = 0x00;
					send = 1;
				}
		//-
		else if (receivedData[0] == ASCII_DASH) {
					keyboardHID.key1 = KB_DASH;
					keyboardHID.modifiers = 0x00;
					send = 1;
				}
		//_
		else if (receivedData[0] == ASCII_UNDERSCORE) {
					keyboardHID.key1 = KB_DASH;
					keyboardHID.modifiers = 0x02;
					send = 1;
				}
		//+
		else if (receivedData[0] == ASCII_PLUS){
					keyboardHID.key1 = KB_EQUALS;
					keyboardHID.modifiers = 0x02;
					send = 1;
				}
		//=
		else if (receivedData[0] == ASCII_EQUALS) {
					keyboardHID.key1 = KB_EQUALS;
					keyboardHID.modifiers = 0x00;
					send = 1;
				}

		//Up arrow
#ifdef OSX
		else if (receivedData[0] == 0xe2 && receivedData[1] == 0x88
				&& receivedData[2] == 0x91) {

#else
		else if (receivedData[0] == 0x1b && receivedData[1] == ASCII_w) {
#endif
			keyboardHID.key1 = KB_UP;
			keyboardHID.modifiers = 0;
			send = 1;
		}

		//Left arrow
#ifdef OSX
		else if (receivedData[0] == 0xc3 && receivedData[1] == 0xa5) {

#else
		else if (receivedData[0] == 0x1b && receivedData[1] == ASCII_a) {
#endif
			keyboardHID.key1 = KB_LEFT;
			keyboardHID.modifiers = 0;
			send = 1;
		}
		//Down arrow

#ifdef OSX
		else if (receivedData[0] == 0xc3 && receivedData[1] == 0x9f) {

#else
		else if (receivedData[0] == 0x1b && receivedData[1] == ASCII_s) {

#endif
			keyboardHID.key1 = KB_DOWN;
			keyboardHID.modifiers = 0;
			send = 1;
		}
		//right arrow

#ifdef OSX
		else if (receivedData[0] == 0xe2 && receivedData[1] == 0x88
				&& receivedData[2] == 0x82) {

#else
		else if (receivedData[0] == 0x1b && receivedData[1] == ASCII_d) {

#endif
			keyboardHID.key1 = KB_RIGHT;
			keyboardHID.modifiers = 0;
			send = 1;
		}
		//Backspace
#ifdef OSX
		else if (receivedData[0] == 0xc5 && receivedData[1] == 0x93) {
#else
		else if (receivedData[0] == 0x1b && receivedData[1] == ASCII_q) {
#endif

			keyboardHID.key1 = KB_BACKSPC;
			keyboardHID.modifiers = 0;
			send = 1;
		}

#ifdef OSX
		//TODO: Add delete for OSX
#else
		else if (receivedData[0] == 0x1b && receivedData[1] == ASCII_e) {
#endif
			keyboardHID.key1 = KB_DELETE;
			keyboardHID.modifiers = 0;
			send = 1;
		}
#ifdef wat
		if (receivedData[0] != 0) {
			keyboardHID.key1 = 0;
			keyboardHID.modifiers = 0;
			send = 1;

		}
#endif

		//Only send report if a key has been pressed
		if (send == 1) {
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID,
					sizeof(struct keyboardHID_t));
			HAL_Delay(30);
			keyboardHID.key1 = 0;
			keyboardHID.modifiers = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID,
					sizeof(struct keyboardHID_t));
			send = 0;
		}
	}

	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART3 init function */
static void MX_USART3_UART_Init(void) {

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins
 PE4   ------> SAI1_FS_A
 PG14   ------> USART6_TX
 PE1   ------> FMC_NBL1
 PE0   ------> FMC_NBL0
 PB8   ------> I2C1_SCL
 PB3   ------> I2S3_CK
 PC12   ------> SDIO_CK
 PE5   ------> SAI1_SCK_A
 PB9   ------> I2C1_SDA
 PB6   ------> QUADSPI_BK1_NCS
 PG15   ------> FMC_SDNCAS
 PD6   ------> SAI1_SD_A
 PD0   ------> FMC_D2_DA2
 PC11   ------> SDIO_D3
 PC10   ------> SDIO_D2
 PI4   ------> FMC_NBL2
 PD1   ------> FMC_D3_DA3
 PI3   ------> FMC_D27
 PI2   ------> FMC_D26
 PF0   ------> FMC_A0
 PI5   ------> FMC_NBL3
 PI7   ------> FMC_D29
 PI10   ------> FMC_D31
 PI6   ------> FMC_D28
 PG9   ------> USART6_RX
 PD2   ------> SDIO_CMD
 PH15   ------> FMC_D23
 PI1   ------> FMC_D25
 PF1   ------> FMC_A1
 PI9   ------> FMC_D30
 PH13   ------> FMC_D21
 PH14   ------> FMC_D22
 PI0   ------> FMC_D24
 PC9   ------> SDIO_D1
 PF2   ------> FMC_A2
 PC8   ------> SDIO_D0
 PF3   ------> FMC_A3
 PH4   ------> I2C2_SCL
 PG8   ------> FMC_SDCLK
 PF4   ------> FMC_A4
 PH5   ------> I2C2_SDA
 PH3   ------> FMC_SDNE0
 PG7   ------> SAI1_MCLK_A
 PF7   ------> QUADSPI_BK1_IO2
 PF6   ------> QUADSPI_BK1_IO3
 PF5   ------> FMC_A5
 PH2   ------> FMC_SDCKE0
 PD15   ------> FMC_D1_DA1
 PD10   ------> FMC_D15_DA15
 PF10   ------> QUADSPI_CLK
 PF9   ------> QUADSPI_BK1_IO1
 PF8   ------> QUADSPI_BK1_IO0
 PD14   ------> FMC_D0_DA0
 PD9   ------> FMC_D14_DA14
 PD8   ------> FMC_D13_DA13
 PC0   ------> FMC_SDNWE
 PF12   ------> FMC_A6
 PG1   ------> FMC_A11
 PF15   ------> FMC_A9
 PD13   ------> S_TIM4_CH2
 PH12   ------> FMC_D20
 PF13   ------> FMC_A7
 PG0   ------> FMC_A10
 PE8   ------> FMC_D5_DA5
 PG5   ------> FMC_A15_BA1
 PG4   ------> FMC_A14_BA0
 PH9   ------> FMC_D17
 PH11   ------> FMC_D19
 PF14   ------> FMC_A8
 PJ2   ------> DSIHOST_TE
 PF11   ------> FMC_SDNRAS
 PE9   ------> FMC_D6_DA6
 PE11   ------> FMC_D8_DA8
 PE14   ------> FMC_D11_DA11
 PH8   ------> FMC_D16
 PH10   ------> FMC_D18
 PE7   ------> FMC_D4_DA4
 PE10   ------> FMC_D7_DA7
 PE12   ------> FMC_D9_DA9
 PE15   ------> FMC_D12_DA12
 PE13   ------> FMC_D10_DA10
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOG_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOI_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOK_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOJ_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, SPKR_HP_Pin | AUDIO_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, LED3_Pin | LED2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, OTG_FS1_PowerSwitchOn_Pin | EXT_RESET_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : SAI1_FSA_Pin SAI1_SCKA_Pin */
	GPIO_InitStruct.Pin = SAI1_FSA_Pin | SAI1_SCKA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : SPKR_HP_Pin AUDIO_RST_Pin */
	GPIO_InitStruct.Pin = SPKR_HP_Pin | AUDIO_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : ARDUINO_USART6_TX_Pin USART6_RX_Pin */
	GPIO_InitStruct.Pin = ARDUINO_USART6_TX_Pin | USART6_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : FMC_NBL1_Pin FMC_NBL0_Pin D5_Pin D6_Pin
	 D8_Pin D11_Pin D4_Pin D7_Pin
	 D9_Pin D12_Pin D10_Pin */
	GPIO_InitStruct.Pin = FMC_NBL1_Pin | FMC_NBL0_Pin | D5_Pin | D6_Pin | D8_Pin
			| D11_Pin | D4_Pin | D7_Pin | D9_Pin | D12_Pin | D10_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
	GPIO_InitStruct.Pin = I2C1_SCL_Pin | I2C1_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : I2S3_CK_Pin */
	GPIO_InitStruct.Pin = I2S3_CK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(I2S3_CK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : uSD_CLK_Pin uSD_D3_Pin uSD_D2_Pin uSD_D1_Pin
	 uSD_D0_Pin */
	GPIO_InitStruct.Pin = uSD_CLK_Pin | uSD_D3_Pin | uSD_D2_Pin | uSD_D1_Pin
			| uSD_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS1_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS1_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS1_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : QSPI_BK1_NCS_Pin */
	GPIO_InitStruct.Pin = QSPI_BK1_NCS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_QSPI;
	HAL_GPIO_Init(QSPI_BK1_NCS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SDNCAS_Pin SDCLK_Pin A11_Pin A10_Pin
	 PG5 PG4 */
	GPIO_InitStruct.Pin = SDNCAS_Pin | SDCLK_Pin | A11_Pin | A10_Pin
			| GPIO_PIN_5 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : MIC_DATA_Pin */
	GPIO_InitStruct.Pin = MIC_DATA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
	HAL_GPIO_Init(MIC_DATA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : D2_Pin D3_Pin D1_Pin D15_Pin
	 D0_Pin D14_Pin D13_Pin */
	GPIO_InitStruct.Pin = D2_Pin | D3_Pin | D1_Pin | D15_Pin | D0_Pin | D14_Pin
			| D13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : FMC_NBL2_Pin D27_Pin D26_Pin FMC_NBL3_Pin
	 D29_Pin D31_Pin D28_Pin D25_Pin
	 D30_Pin D24_Pin */
	GPIO_InitStruct.Pin = FMC_NBL2_Pin | D27_Pin | D26_Pin | FMC_NBL3_Pin
			| D29_Pin | D31_Pin | D28_Pin | D25_Pin | D30_Pin | D24_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pins : LED3_Pin LED2_Pin */
	GPIO_InitStruct.Pin = LED3_Pin | LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : A0_Pin A1_Pin A2_Pin A3_Pin
	 A4_Pin A5_Pin A6_Pin A9_Pin
	 A7_Pin A8_Pin SDNMT48LC4M32B2B5_6A_RAS_RAS___Pin */
	GPIO_InitStruct.Pin =
	A0_Pin | A1_Pin | A2_Pin | A3_Pin | A4_Pin | A5_Pin | A6_Pin | A9_Pin
			| A7_Pin | A8_Pin | SDNMT48LC4M32B2B5_6A_RAS_RAS___Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : LED4_Pin */
	GPIO_InitStruct.Pin = LED4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : uSD_CMD_Pin */
	GPIO_InitStruct.Pin = uSD_CMD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
	HAL_GPIO_Init(uSD_CMD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : D23_Pin D21_Pin D22_Pin SDNE0_Pin
	 SDCKE0_Pin D20_Pin D17_Pin D19_Pin
	 D16_Pin D18_Pin */
	GPIO_InitStruct.Pin = D23_Pin | D21_Pin | D22_Pin | SDNE0_Pin | SDCKE0_Pin
			| D20_Pin | D17_Pin | D19_Pin | D16_Pin | D18_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : I2C2_SCL_Pin I2C2_SDA_Pin */
	GPIO_InitStruct.Pin = I2C2_SCL_Pin | I2C2_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : SAI1_MCLKA_Pin */
	GPIO_InitStruct.Pin = SAI1_MCLKA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
	HAL_GPIO_Init(SAI1_MCLKA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED1_Pin */
	GPIO_InitStruct.Pin = LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : QSPI_BK1_IO2_Pin QSPI_BK1_IO3_Pin QSPI_CLK_Pin */
	GPIO_InitStruct.Pin = QSPI_BK1_IO2_Pin | QSPI_BK1_IO3_Pin | QSPI_CLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_QSPI;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : QSPI_BK1_IO1_Pin QSPI_BK1_IO0_Pin */
	GPIO_InitStruct.Pin = QSPI_BK1_IO1_Pin | QSPI_BK1_IO0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_QSPI;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : SDNWE_Pin */
	GPIO_InitStruct.Pin = SDNWE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	HAL_GPIO_Init(SDNWE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS1_PowerSwitchOn_Pin EXT_RESET_Pin */
	GPIO_InitStruct.Pin = OTG_FS1_PowerSwitchOn_Pin | EXT_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : MIC_CK_Pin */
	GPIO_InitStruct.Pin = MIC_CK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(MIC_CK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : uSD_Detect_Pin */
	GPIO_InitStruct.Pin = uSD_Detect_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_INT_Pin */
	GPIO_InitStruct.Pin = LCD_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DSI_TE_Pin */
	GPIO_InitStruct.Pin = DSI_TE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF13_DSI;
	HAL_GPIO_Init(DSI_TE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_BL_CTRL_Pin */
	GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
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
