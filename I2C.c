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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADS7138_ADDR7      0x13   // 7-bit I²C address (page19 from ads7138-q1) 0x17 to 0x10 are the possible device addresses for I2C
#define SAMPLE_CHANNEL     0x00   // which channel to read from **change it** based on which pin your sun sensor signal is wired to

#define REG_SYSTEM_STATUS  0x00    // System status
#define REG_GENERAL_CFG    0x01  // controls reset, standby, data formatting, and manual conversion start
#define REG_DATA_CFG       0x02  // controls how conversion results are aligned and how channel/status bytes are appended
#define REG_OPMODE_CFG     0x04  // used for picking operating mode
#define REG_PIN_CFG        0x05    // Pin configuration
#define REG_SEQUENCE_CFG   0x10  // only if using auto/sequence mode, otherwise can be ignored
#define REG_MANUAL_CH_SEL  0x11  // tells the ADC which single channel to measure next


// Opcodes for I²C commands 
#define OPCODE_WRITE_SINGLE    0x08    // Single register write 
#define OPCODE_READ_SINGLE     0x10    // Single register read 
#define OPCODE_READ_CONTINUOUS 0x30   // Read a continuous block of register


/* Those registers are the basic necessary ones. Check page 35 if you think you need more */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef ads_write(uint8_t reg, uint8_t val);

static HAL_StatusTypeDef ads_read_frame(uint8_t *buf, uint16_t nbytes);

static uint8_t i2c_scan_find(uint8_t want_addr7);

static void ads_init_manual_mode(void);

__weak void adcs_send(uint8_t *data, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef ads_write(uint8_t reg, uint8_t val){ // create a 3 bytes buffer
    uint8_t tx_buf[3];
    tx_buf[0] = OPCODE_WRITE_SINGLE; // Opcode for single register write 
    tx_buf[1] = reg;                 // Register address 
    tx_buf[2] = val;                 // Data to write 
    return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(ADS7138_ADDR7 << 1), tx_buf, 3, 50);
}


static HAL_StatusTypeDef ads_read_frame(uint8_t *buf, uint16_t nbytes){  //read a “frame” of bytes back from the ADS7138   Returns HAL_OK on success, otherwise an error
    return HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(ADS7138_ADDR7<<1), buf, nbytes, 50);
}

static uint8_t i2c_scan_find(uint8_t want_addr7){  //scan the I²C bus and report if a device responds at the 7-bit address
    for(uint8_t a=1; a<0x7F; a++)
        if(HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a<<1), 1, 5)==HAL_OK)
            if(a==want_addr7) return 1;
    return 0;
}

/*static void ads_init_manual_mode(void){  // Stop any ongoing sequencing first
    
    ads_write_reg(REG_SEQUENCE_CFG, 0x00); 

    // The PIN_CFG register reset value is 0x00, which configures all pins as analog inputs [cite: 1785]
    ads_write_reg(REG_PIN_CFG, 0x00); 
	
	ads_write_reg(REG_GENERAL_CFG, 0x02); // Set CAL bit to 1b 
    while((HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(ADS7138_ADDR7 << 1), REG_GENERAL_CFG, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0}, 1, 50) == HAL_OK) && (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(ADS7138_ADDR7 << 1), (uint8_t[]){0}, 1, 50) == HAL_OK) && ( (uint8_t){0}[0] & 0x02 )); // Wait for CAL bit to be cleared to 0b by the device 

    // Set the operating mode to Manual mode (CONV_MODE=00b, SEQ_MODE=00b)
    // These are the default reset values, so explicit write might be optional but is good practiceö
    ads_write_reg(REG_OPMODE_CFG, 0x00); 
    ads_write_reg(REG_SEQUENCE_CFG, 0x00); 
    // Select the channel to be converted in manual mode
    ads_write_reg(REG_MANUAL_CH_SEL, (uint8_t)(SAMPLE_CHANNEL & 0x0F)); 

    // Configure data output format for 12-bit data with no appended info
    // This is the default reset value, so this line is for clarity and can be omitted.
    ads_write_reg(REG_DATA_CFG, 0x00); [cite: 1757]
}
 */
static void ads_init_manual_mode(void){  //put ADC into manual sequence mode tells the ADC not to loop over channels, only measure the one I say
    ads_write(REG_DATA_CFG, 0x10);
    ads_write(REG_SEQUENCE_CFG, 0x00);
    ads_write(REG_MANUAL_CH_SEL, (uint8_t)(SAMPLE_CHANNEL & 0x0F));
}
__weak void adcs_send(uint8_t *data, uint16_t len){
	(void)data; 
	(void)len; 
}  //default implementation
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  if(!i2c_scan_find(ADS7138_ADDR7)) Error_Handler();
  ads_init_manual_mode();
  uint8_t frame[2];  //change it according to manual

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 // ads_write(REG_MANUAL_CH_SEL, (uint8_t)(SAMPLE_CHANNEL & 0x0F));     //Set the ADC to measure the channel wanted
	  if(ads_read_frame(frame, sizeof frame)==HAL_OK){
		  
	      uint16_t raw12 = (uint16_t)(((uint16_t)frame[0]<<8)|frame[1]); 
	      raw12 >>= 4;  
	      adcs_send((uint8_t*)&raw12, sizeof raw12);
	  }
	  HAL_Delay(10);  // delay time **change it** if you want more


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
