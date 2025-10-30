/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define MAX_REG_DIGITS 11
#define KEYPAD_EMPTY   255
#define SUCCESS_LED_MASK 0b11111110
#define CLOSED_LED 0b11111111
#define LOCKED_LED 0b11110001

struct _ButMtx_Struct {
	GPIO_TypeDef *Port;
	uint16_t Pin;
};

struct _ButMtx_Struct BMX_L[4] = {
		{ GPIOA, GPIO_PIN_9 },
		{ GPIOC, GPIO_PIN_7 },
		{ GPIOB, GPIO_PIN_6 },
		{ GPIOA, GPIO_PIN_7 } };

struct _ButMtx_Struct BMX_R[3] = {
		{ GPIOB, GPIO_PIN_4 },
		{ GPIOB, GPIO_PIN_10 },
		{ GPIOA, GPIO_PIN_8 } };

char Register_Number[MAX_REG_DIGITS + 1] = { 0 };
const char TARGET_NUMBER[MAX_REG_DIGITS + 1] = "67340500031";
uint8_t Current_Digit_Index = 0;
uint16_t ButtonState = 0;
uint32_t LastButtonState = 0;

// TRIAL-OF-ERROR LOCKOUT VARIABLES
uint8_t Failed_Tries = 0;      // Counts failed attempts (0 to 3)
uint8_t is_locked = 0;         // 0: Unlocked, 1: Locked
uint8_t Failed_Try_Mask = 0b11111111;   // Bitmask for error indicator LEDs (e.g., Bit 0, Bit 1, Bit 2)

const char Keypad_Map[12] = {
		'7', '4', '1', '0', // Column 0 (Bits 0-3): 7, 4, 1, 0
		'8', '5', '2', '\0',   // Column 1 (Bits 4-7): 8, 5, 2, (EMPTY/NULL)
		'9', '6', '3', 'K'     // Column 2 (Bits 8-11): 9, 6, 3, 'K' (for OK)
		};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void ButtonMatrixRead();
uint8_t Keypad_Get_Key(void);
void Register_Number_Handler(void);
uint8_t Check_Register_Number(void);
void Clear_Register_to_Zeros(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HC595_Write(uint8_t data_to_send);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define HC595_DATA_PORT       GPIOA
#define HC595_DATA_PIN        GPIO_PIN_0      // SER (DS) -> PA0

#define HC595_LATCH_PORT      GPIOA
#define HC595_LATCH_PIN       GPIO_PIN_1      // RCLK (ST_CP) -> PA1

#define HC595_CLOCK_PORT      GPIOA
#define HC595_CLOCK_PIN       GPIO_PIN_4      // SRCLK (SH_CP) -> PA4

void HC595_Write(uint8_t data_to_send) {
	// 1. Latch LOW: Prepare the Latch register to receive new data
	HAL_GPIO_WritePin(HC595_LATCH_PORT, HC595_LATCH_PIN, GPIO_PIN_RESET);

	// 2. Shift Data (MSB First)
	for (int i = 0; i < 8; i++) {
		// Clock LOW: Prepare for a new bit
		HAL_GPIO_WritePin(HC595_CLOCK_PORT, HC595_CLOCK_PIN, GPIO_PIN_RESET);

		// Set Data Pin (SER) based on the current bit (7-i for MSB First)
		//THIS IS AND GATE of 1
		if ((data_to_send >> (7 - i)) & 0x01) {
			HAL_GPIO_WritePin(HC595_DATA_PORT, HC595_DATA_PIN, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(HC595_DATA_PORT, HC595_DATA_PIN, GPIO_PIN_RESET);
		}

		// Clock HIGH: Shift the set bit into the 74HC595
		HAL_GPIO_WritePin(HC595_CLOCK_PORT, HC595_CLOCK_PIN, GPIO_PIN_SET);

	}

	// 3. Latch HIGH: Transfer the shifted data from the Shift Register to the Storage Register
	// This updates the outputs (LEDs)
	HAL_GPIO_WritePin(HC595_LATCH_PORT, HC595_LATCH_PIN, GPIO_PIN_SET);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

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
	/* USER CODE BEGIN 2 */
	HC595_Write(CLOSED_LED);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		static uint32_t BTMX_TimeStamp = 0;
		if (HAL_GetTick() > BTMX_TimeStamp) {
			BTMX_TimeStamp = HAL_GetTick() + 25;
			ButtonMatrixRead();

			// Only process input if the system is not locked
			if (is_locked == 0) {
				Register_Number_Handler();
			}
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | LD2_Pin | GPIO_PIN_8,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA4 LD2_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LPUART1_TX_Pin LPUART1_RX_Pin */
	GPIO_InitStruct.Pin = LPUART1_TX_Pin | LPUART1_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA7 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB4 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (is_locked == 1) {
		return;
	}

	if (GPIO_Pin == GPIO_PIN_5) {
		Clear_Register_to_Zeros();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HC595_Write(CLOSED_LED);
	}
}

uint8_t Keypad_Get_Key(void) {
	uint32_t current_state = ButtonState;
	uint32_t new_press = current_state & (~LastButtonState);
	LastButtonState = current_state;

	if (new_press != 0) {
		return (uint8_t) __builtin_ctz(new_press);
	}

	return 255; // No new key pressed
}

uint8_t Check_Register_Number(void) {
	// 1. Check Length: The input MUST be exactly 11 digits long.
	if (Current_Digit_Index != MAX_REG_DIGITS) {
		return 0; // Wrong length
	}

	// 2. Compare Content: Loop through the array to check each character.
	for (uint8_t i = 0; i < MAX_REG_DIGITS; i++) {
		if (Register_Number[i] != TARGET_NUMBER[i]) {
			return 0; // Mismatch found
		}
	}

	// 3. Match: If the loop completes, the numbers are identical.
	return 1;
}

void ButtonMatrixRead() {
	static uint8_t X = 0;

	for (int i = 0; i < 4; i++) {
		uint8_t bit_pos = i + (X * 4);

		if (HAL_GPIO_ReadPin(BMX_L[i].Port, BMX_L[i].Pin) == GPIO_PIN_RESET) {
			ButtonState |= 1 << (bit_pos);
		}

		else {
			ButtonState &= ~(1 << bit_pos);
		}
	}

	HAL_GPIO_WritePin(BMX_R[X].Port, BMX_R[X].Pin, GPIO_PIN_SET);
	uint8_t nextX = (X + 1) % 3;
	HAL_GPIO_WritePin(BMX_R[nextX].Port, BMX_R[nextX].Pin, GPIO_PIN_RESET);
	X = nextX;
}

void Clear_Register_to_Zeros(void) {
	for (uint8_t i = 0; i < MAX_REG_DIGITS; i++) {
		Register_Number[i] = '0';
	}
	Current_Digit_Index = 0;
	Register_Number[MAX_REG_DIGITS] = '\0'; // Ensure termination is correct
}

void Register_Number_Handler(void) {
	if (is_locked == 1) {
		return;
	}

	uint8_t key_index = Keypad_Get_Key();
	if (key_index != KEYPAD_EMPTY && key_index < 12) {
		char pressed_char = Keypad_Map[key_index];
		if (pressed_char == '\0') {
			return;
		}

		if (pressed_char >= '0' && pressed_char <= '9') {

			if (Current_Digit_Index < MAX_REG_DIGITS) {
				Register_Number[Current_Digit_Index++] = pressed_char;
				Register_Number[Current_Digit_Index] = '\0';
			}
		}

		else if (pressed_char == 'K') {

			if (Current_Digit_Index > 0) {

				uint8_t match = Check_Register_Number();

				if (match == 1) {
					Failed_Tries = 0;
					Failed_Try_Mask = 0b11111111;
					HC595_Write(SUCCESS_LED_MASK);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				}

				else {
					Failed_Tries++;

					if (Failed_Tries <= 3) {
						Failed_Try_Mask &= ~(1 << Failed_Tries);
						HC595_Write(Failed_Try_Mask);
					}

					if (Failed_Tries >= 3) {
						is_locked = 1;
						HC595_Write(LOCKED_LED);
					}

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
					Current_Digit_Index = 0;
					Register_Number[0] = '\0';
				}
			}
		}
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
