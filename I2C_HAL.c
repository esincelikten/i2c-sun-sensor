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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h> // For sprintf
#include <string.h> // For strlen
#include <math.h> // For sqrt()
#include "sunsensorestimation1.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define   (Register addresses)----------------------------------*/
/* USER CODE BEGIN PD */

#define ADR          (0x13 << 1) // 7-bit I2C address shifted left for 8-bit HAL address
#define VREF         3.300f


#define REG_SYSTEM_STATUS  0x00
#define REG_GENERAL_CFG    0x01
#define REG_DATA_CFG       0x02
#define REG_OSR_CFG        0x03
#define REG_OPMODE_CFG     0x04
#define REG_PIN_CFG        0x05
#define REG_SEQUENCE_CFG   0x10
#define REG_MANUAL_CH_SEL  0x11


#define RECENT_BASE_LSB 0xA0


// Command opcodes

#define OPC_READ_REG_1   0x10
#define OPC_WRITE_REG_1  0x08
#define OPC_READ_BLOCK   0x30
#define OPC_SET_BITS     0x18
#define OPC_CLEAR_BITS   0x20

// Timeout values (ms)

#define I2C_TIMEOUT_MS   100 // Timeout for I2C operations
#define UART_TX_TIMEOUT_MS 100 // Timeout for UART transmit


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//Global Accessibility variables

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Custom UART print function for debugging
void print_string(const char *str);
void print_float(float val, int precision);
void print_hex(uint8_t val);
void print_new_line();

//Helper functions

/* Here I didn't use incline cause it is not low level and compiler in STM32 is very modern */

static bool regWrite(uint8_t reg, uint8_t val);

static bool regRead1(uint8_t reg, uint8_t *val);

static bool regReadN(uint8_t startReg, uint8_t *buf, uint8_t n);

static bool regSetBits(uint8_t reg, uint8_t mask);

static bool regClearBits(uint8_t reg, uint8_t mask);

// Convert Helpers

static bool readRecent12(uint8_t ch, uint16_t *code12);

//CHO Temperature Function
float voltageToTempC(float V_meas);

//Manuel Configuration Function
void configureManual(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  configureManual();

  uint8_t sys=0;

  char message_buffer[100]; // Buffer for serial messages


  if (regRead1(REG_SYSTEM_STATUS, &sys)) {
      sprintf(message_buffer, "SYSTEM_STATUS=0x%02X\r\n", sys);
      print_string(message_buffer);
    } else {
      print_string("SYSTEM_STATUS read error\r\n");
    }

    print_string("Manual per-channel conversions (CH0..CH5)\r\n");

    sunsensorestimation1_initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  double y[5];    // CH1–CH5 değerleri
	  double result[3];

	  for (uint8_t ch = 0; ch <= 5; ch++)
	  {
		  uint32_t sum = 0;
		  uint16_t code;

		  for (uint8_t i = 0; i < 10; i++)
		  {
			  regWrite(REG_MANUAL_CH_SEL, (uint8_t)(ch));
			  regSetBits(REG_GENERAL_CFG, 0x08);
			  HAL_Delay(2); // delayMicroseconds(2000) -> approximately HAL_Delay(2)

			  if (readRecent12(ch, &code))
			  {
				  sum += code;
			  }
		  uint16_t avgCode = sum / 10;
		  float volts = (avgCode / 4095.0f) * VREF;

		  if (ch >= 1 && ch <= 5)
		  {
			  y[ch - 1] = (double)volts;
		  }

		  }
	  sunsensorestimation1(y, result);
	  print_string("sunsensorestimation1(y) = [");

	  for (int i = 0; i < 3; i++)
	  {
		  sprintf(msg_buffer, "%.4f", result[i]);
		  print_string(msg_buffer);
		  if (i < 2) print_string(" ");
	  }
	  print_string("]\r\n");

	  HAL_Delay(5000); // delay(5000)
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static bool regWrite(uint8_t reg, uint8_t val) {
  uint8_t tx_buffer[3];
  tx_buffer[0] = OPC_WRITE_REG_1;
  tx_buffer[1] = reg;
  tx_buffer[2] = val;
  if (HAL_I2C_Master_Transmit(&hi2c1, ADR, tx_buffer, 3, I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  return true;
}

static bool regRead1(uint8_t reg, uint8_t *val) {
  uint8_t tx_buffer[2];
  tx_buffer[0] = OPC_READ_REG_1;
  tx_buffer[1] = reg;
  if (HAL_I2C_Master_Transmit(&hi2c1, ADR, tx_buffer, 2, I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  if (HAL_I2C_Master_Receive(&hi2c1, ADR, val, 1, I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  return true;
}


static bool regReadN(uint8_t startReg, uint8_t *buf, uint8_t n) {
  uint8_t tx_buffer[2];
  tx_buffer[0] = OPC_READ_BLOCK;
  tx_buffer[1] = startReg;
  if (HAL_I2C_Master_Transmit(&hi2c1, ADR, tx_buffer, 2, I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  if (HAL_I2C_Master_Receive(&hi2c1, ADR, buf, n, I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  return true;
}


static bool regSetBits(uint8_t reg, uint8_t mask) {
  uint8_t tx_buffer[3];
  tx_buffer[0] = OPC_SET_BITS;
  tx_buffer[1] = reg;
  tx_buffer[2] = mask;
  if (HAL_I2C_Master_Transmit(&hi2c1, ADR, tx_buffer, 3, I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  return true;
}


static bool regClearBits(uint8_t reg, uint8_t mask) {
  uint8_t tx_buffer[3];
  tx_buffer[0] = OPC_CLEAR_BITS;
  tx_buffer[1] = reg;
  tx_buffer[2] = mask;
  if (HAL_I2C_Master_Transmit(&hi2c1, ADR, tx_buffer, 3, I2C_TIMEOUT_MS) != HAL_OK) {
    return false;
  }
  return true;
}


static bool readRecent12(uint8_t ch, uint16_t *code12) {
  uint8_t buf[2];
  uint8_t lsbAddr = RECENT_BASE_LSB + (ch * 2);
  if (!regReadN(lsbAddr, buf, 2)) return false;
  uint16_t raw16 = ((uint16_t)buf[1] << 8) | buf[0];
  *code12 = (raw16 >> 4) & 0x0FFF;
  return true;
}


float voltageToTempC(float V_meas) {
  // V_meas mV cinsinden olmalı
  float a = -0.00433f;
  float b = -13.582f;
  float c = 2230.8f - V_meas;

  float D = b * b - 4 * a * c;
  if (D < 0) return -999.0f; // geçersiz

  float x1 = (-b + sqrtf(D)) / (2 * a); // Use sqrtf for float
  float x2 = (-b - sqrtf(D)) / (2 * a);

  float T1 = 30 + x1;
  float T2 = 30 + x2;

  if (T1 > -40 && T1 < 125) return T1;
  else return T2;
}


void configureManual(void) {
  regSetBits(REG_GENERAL_CFG, 0x01);
  HAL_Delay(2); // delay(2)

  regWrite(REG_PIN_CFG, 0x00);
  regWrite(REG_DATA_CFG, 0x00);
  regWrite(REG_OSR_CFG,  0x00);
  regWrite(REG_OPMODE_CFG, 0x00);
  regWrite(REG_SEQUENCE_CFG, 0x00);
  regSetBits(REG_GENERAL_CFG, 0x20);
}


void print_string(const char *str) {
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), UART_TX_TIMEOUT_MS);
}

void print_float(float val, int precision) {
    char buffer[32]; // Buffer to hold the float string
    sprintf(buffer, "%.*f", precision, val);
    print_string(buffer);
}

void print_hex(uint8_t val) {
    char buffer[10];
    sprintf(buffer, "0x%02X", val);
    print_string(buffer);
}

void print_new_line() {
    print_string("\r\n");
}
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
