/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body - Improved PPG Heart Rate Detection with BPM Averaging
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
#include "math.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 100
#define FILTER_SIZE 5
#define MIN_PEAK_HEIGHT 50
#define MIN_PEAK_DISTANCE 200
#define SIGNAL_THRESHOLD 30
#define BPM_AVERAGE_SIZE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
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

// Enhanced filtering with moving average
uint32_t apply_moving_average(uint32_t new_sample, uint32_t *filter_buffer, int *filter_index) {
    filter_buffer[*filter_index] = new_sample;
    *filter_index = (*filter_index + 1) % FILTER_SIZE;

    uint64_t sum = 0;
    for(int i = 0; i < FILTER_SIZE; i++) {
        sum += filter_buffer[i];
    }
    return sum / FILTER_SIZE;
}

// Function to calculate BPM average
float calculate_bpm_average(float *bpm_array, int count) {
    if (count == 0) return 0.0;

    float sum = 0.0;
    for (int i = 0; i < count; i++) {
        sum += bpm_array[i];
    }
    return sum / count;
}

/* USER CODE END 0 */

/**
* @brief The application entry point.
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
    HAL_Delay(100); // Allow reset to complete
    max30102_clear_fifo(&max30102);

    // FIFO configuration
    max30102_set_fifo_config(&max30102, max30102_smp_ave_4, 1, 15);

    // Sensor settings
    max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
    max30102_set_adc_resolution(&max30102, max30102_adc_4096);
    max30102_set_sampling_rate(&max30102, max30102_sr_100);

    // LED current settings
    max30102_set_led_current_1(&max30102, 10.0);
    max30102_set_led_current_2(&max30102, 10.0);

    // Enter SpO2 mode
    max30102_set_mode(&max30102, max30102_spo2);
    max30102_set_a_full(&max30102, 1);
    max30102_set_ppg_rdy(&max30102, 1);

    // Wait for sensor to start
    HAL_Delay(1000);

    // Signal processing variables
    uint32_t ir_buffer[BUFFER_SIZE] = {0};
    int buffer_index = 0;
    uint32_t filtered_buffer[FILTER_SIZE] = {0};
    int filter_index = 0;

    // Peak detection variables
    uint32_t last_peak_time = 0;
    float current_bpm = 0;
    int32_t prev_signal = 0;
    int32_t prev_prev_signal = 0;
    bool peak_found = false;

    // BPM averaging variables
    float bpm_array[BPM_AVERAGE_SIZE];
    int bpm_counter = 0;
    float average_bpm = 0.0;
    bool array_filled = false;

    // Initialize BPM array
    for (int i = 0; i < BPM_AVERAGE_SIZE; i++) {
        bpm_array[i] = 0.0;
    }

    // Startup counter to fill buffer
    uint32_t startup_samples = 0;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // Read FIFO
        max30102_read_fifo(&max30102);
        uint32_t ir_value = max30102._ir_samples[0];

        // Apply moving average filter
        uint32_t filtered_value = apply_moving_average(ir_value, filtered_buffer, &filter_index);

        // Store in circular buffer for baseline calculation
        ir_buffer[buffer_index++] = filtered_value;
        if (buffer_index >= BUFFER_SIZE) buffer_index = 0;

        // Skip processing until buffer is filled
        if (startup_samples < BUFFER_SIZE) {
            startup_samples++;
            HAL_Delay(10);
            continue;
        }

        // Calculate baseline as average
        uint64_t sum = 0;
        for (int i = 0; i < BUFFER_SIZE; i++) {
            sum += ir_buffer[i];
        }
        uint32_t baseline = sum / BUFFER_SIZE;

        // AC component (signal minus baseline)
        int32_t ir_ac = (int32_t)filtered_value - (int32_t)baseline;

        uint32_t current_time = HAL_GetTick();

        // 3-point peak detection
        if (ir_ac > prev_signal && prev_signal > prev_prev_signal &&
            ir_ac > SIGNAL_THRESHOLD && !peak_found) {

            // Peak detected at previous point
            if (current_time - last_peak_time > MIN_PEAK_DISTANCE) {

                // Calculate BPM
                if (last_peak_time > 0) {
                    uint32_t interval = current_time - last_peak_time;
                    current_bpm = 60000.0 / interval;

                    // Validate BPM range (40-200 BPM)
                    if (current_bpm >= 40 && current_bpm <= 200) {

                        // Add to BPM array for averaging
                        bpm_array[bpm_counter] = current_bpm;
                        bpm_counter++;

                        // Check if array is full
                        if (bpm_counter >= BPM_AVERAGE_SIZE) {
                            bpm_counter = 0;
                            array_filled = true;
                        }

                        // Calculate average BPM
                        int count_for_average = array_filled ? BPM_AVERAGE_SIZE : bpm_counter;
                        if (count_for_average > 0) {
                            average_bpm = calculate_bpm_average(bpm_array, count_for_average);
                        }
                    }
                }
                last_peak_time = current_time;
                peak_found = true;
            }
        } else if (ir_ac < prev_signal) {
            peak_found = false;
        }

        // Update signal history
        prev_prev_signal = prev_signal;
        prev_signal = ir_ac;

        // Display on OLED
        ssd1306_Fill(0); // Clear screen

        // Display current signal
        char signal_str[32];
        snprintf(signal_str, sizeof(signal_str), "Signal: %ld", ir_ac);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString(signal_str, Font_7x10, White);

        // Display current BPM
        if (current_bpm > 0) {
            char bpm_str[32];
            snprintf(bpm_str, sizeof(bpm_str), "BPM: %.0f", current_bpm);
            ssd1306_SetCursor(0, 15);
            ssd1306_WriteString(bpm_str, Font_7x10, White);
        }

        // Display average BPM
        if (average_bpm > 0) {
            char avg_bpm_str[32];
            snprintf(avg_bpm_str, sizeof(avg_bpm_str), "Avg: %.1f", average_bpm);
            ssd1306_SetCursor(0, 30);
            ssd1306_WriteString(avg_bpm_str, Font_7x10, White);
        }

        // Display sample count for averaging
        char count_str[32];
        int display_count = array_filled ? BPM_AVERAGE_SIZE : bpm_counter;
        snprintf(count_str, sizeof(count_str), "Samples: %d", display_count);
        ssd1306_SetCursor(0, 45);
        ssd1306_WriteString(count_str, Font_7x10, White);

        ssd1306_UpdateScreen();
        HAL_Delay(10);

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
  * in the RCC_OscInitStruct structure.
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
