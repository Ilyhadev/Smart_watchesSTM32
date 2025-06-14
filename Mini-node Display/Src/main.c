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
#include "ssd1306_fonts.h"
#include "ssd1306.h"
#include "max30102_for_stm32_hal.h"
#include "stdlib.h"
#include <stdio.h>
#include <stdbool.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RATE_SIZE 8
#define BUFFER_SIZE 10
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
// Heart rate calculation variables
uint8_t rates[RATE_SIZE];
uint8_t rateSpot = 0;
uint32_t lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

// Peak detection variables
uint32_t irValue = 0;
uint32_t lastIRValue = 0;
uint32_t delta = 0;
bool fingerDetected = false;

// Moving average for noise reduction
uint32_t irBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;
uint32_t irAverage = 0;
uint32_t lastSample = 0;    // Moved from checkForBeat()
// MAX30102 object
max30102_t max30102;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Simple peak detection function
bool checkForBeat(uint32_t sample) {
    static uint32_t threshold = 100000;

    if (sample < 50000) {
        return false;
    }

    // Dynamic threshold decay (5% per call)
    threshold = threshold * 0.95 + sample * 0.05;

    if (sample > lastSample && sample > threshold) {
        uint32_t currentTime = HAL_GetTick();
        if ((currentTime - lastBeat) > 400) {  // Increased to 400ms
            threshold = sample * 0.7;  // Reduced from 0.8 to 0.7
            lastSample = sample;
            return true;
        }
    }

    lastSample = sample;
    return false;
}

// Calculate moving average for noise reduction
uint32_t calculateMovingAverage(uint32_t newValue) {
    irBuffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    uint32_t sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += irBuffer[i];
    }
    return sum / BUFFER_SIZE;
}
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
  // Initialize MAX30102 and SSD1306
    ssd1306_Init();
    max30102_init(&max30102, &hi2c1);
  max30102_reset(&max30102);
  HAL_Delay(100);

  max30102_clear_fifo(&max30102);
  max30102_set_fifo_config(&max30102, max30102_smp_ave_16, 1, 7);

  // Optimized sensor settings for heart rate detection
  max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
  max30102_set_adc_resolution(&max30102, max30102_adc_4096);
  max30102_set_sampling_rate(&max30102, max30102_sr_100); // Lower sampling rate for HR
  max30102_set_led_current_1(&max30102, 10); // Higher LED current for better signal
  max30102_set_led_current_2(&max30102, 10);

  // Enter SpO2 mode (uses both RED and IR LEDs)
  max30102_set_mode(&max30102, max30102_spo2);
  max30102_set_a_full(&max30102, 1);




  // Display initialization message
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Heart Rate Monitor", Font_7x10, 1);
  ssd1306_SetCursor(0, 15);
  ssd1306_WriteString("Place finger on", Font_7x10, 1);
  ssd1306_SetCursor(0, 25);
  ssd1306_WriteString("sensor...", Font_7x10, 1);
  ssd1306_UpdateScreen();

  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Read sensor data
	      max30102_read_fifo(&max30102);

	      // Get IR value for heart rate detection (IR is more stable than RED)
	      irValue = max30102._ir_samples[0];

	      // Apply moving average filter
	      irAverage = calculateMovingAverage(irValue);

	      // Check for finger presence
	      fingerDetected = (irValue > 50000);

	      // Clear display area
	      ssd1306_SetCursor(0, 0);
	      ssd1306_WriteString("                  ", Font_16x24, 1);
	      ssd1306_SetCursor(0, 15);
	      ssd1306_WriteString("                ", Font_16x24, 1);
	      ssd1306_SetCursor(0, 30);
	      ssd1306_WriteString("                ", Font_11x18, 1);
	      ssd1306_SetCursor(0, 45);
	      ssd1306_WriteString("                ", Font_11x18, 1);

	      if (fingerDetected) {
	          // Display finger detected status
	          ssd1306_SetCursor(0, 0);
	          ssd1306_WriteString("Finger: OK", Font_7x10, 1);

	          // Check for heartbeat
	          if (checkForBeat(irAverage)) {
	              // Calculate time between beats
	              delta = HAL_GetTick() - lastBeat;
	              lastBeat = HAL_GetTick();

	              // Calculate BPM
	              beatsPerMinute = 60.0f / (delta / 1000.0f);

	              // Validate BPM range (normal human range)
	              if (beatsPerMinute < 255 && beatsPerMinute > 20 && beatsPerMinute<145) {
	                  rates[rateSpot++] = (uint8_t)beatsPerMinute;
	                  rateSpot %= RATE_SIZE;

	                  // Calculate average BPM
	                  beatAvg = 0;
	                  for (uint8_t x = 0; x < RATE_SIZE; x++) {
	                      beatAvg += rates[x];
	                  }
	                  beatAvg /= RATE_SIZE;
	              }
	          }

	          // Display current BPM
	          char bpmString[16];
	          sprintf(bpmString, "BPM: %.0f", beatsPerMinute);
	          ssd1306_SetCursor(0, 15);
	          ssd1306_WriteString(bpmString, Font_7x10, 1);

	          // Display average BPM
	          char avgString[16];
	          sprintf(avgString, "Avg: %d", beatAvg);
	          ssd1306_SetCursor(0, 30);
	          ssd1306_WriteString(avgString, Font_7x10, 1);

	          // Display signal strength
	          char signalString[16];
	          sprintf(signalString, "Signal: %lu", irValue / 1000);
	          ssd1306_SetCursor(0, 45);
	          ssd1306_WriteString(signalString, Font_7x10, 1);

	      } else {
	          // No finger detected
	          ssd1306_SetCursor(0, 0);
	          ssd1306_WriteString("No finger", Font_7x10, 1);
	          ssd1306_SetCursor(0, 15);
	          ssd1306_WriteString("detected", Font_7x10, 1);

	          // Reset heart rate values
	          beatsPerMinute = 0;
	          beatAvg = 0;
	      }

	      ssd1306_UpdateScreen();

	      // Handle interrupts
	      if (max30102_has_interrupt(&max30102)) {
	          max30102_interrupt_handler(&max30102);
	      }

	      HAL_Delay(50); // Reduced delay for more responsive detection
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
