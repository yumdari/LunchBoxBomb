/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
bool ws2812Init(void);
void ws2812Begin(uint32_t led_cnt);
void ws2812SetColor(uint32_t index, uint8_t red, uint8_t green, uint8_t blue);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  ws2812Init();
  ws2812Begin(8);

  uint32_t pre_time;
  uint32_t led_index = 0;
  int32_t  led_index_pre = -1;
  uint8_t led_color = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (HAL_GetTick()-pre_time >= 100)
    {
      pre_time = HAL_GetTick();

      if (led_color == 0) ws2812SetColor(led_index, 255, 0, 0);
      if (led_color == 1) ws2812SetColor(led_index, 0, 255, 0);
      if (led_color == 2) ws2812SetColor(led_index, 0, 0, 255);

      if (led_index_pre >= 0)
      {
        ws2812SetColor(led_index_pre, 0, 0, 0);
      }
      led_index_pre = led_index;
      led_index = (led_index + 1) % 8;

      if (led_index == 0)
      {
        led_color = (led_color + 1) % 3;
      }
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


#define BIT_PERIOD      (100)
#define BIT_HIGH        (50)
#define BIT_LOW         (28)
#define BIT_OFFSET       80
#define BIT_WIDTH        32
#define BIT_MAX_COUNT    64


bool is_init = false;


typedef struct
{
  uint16_t led_cnt;
} ws2812_t;

static uint8_t led_buf[BIT_OFFSET + BIT_WIDTH*BIT_MAX_COUNT];


ws2812_t ws2812;
extern TIM_HandleTypeDef htim1;



bool ws2812Init(void)
{
  memset(led_buf, 0, sizeof(led_buf));

  is_init = true;


  return true;
}

void ws2812Begin(uint32_t led_cnt)
{
  ws2812.led_cnt = led_cnt;

  for (int i=0; i<led_cnt; i++)
  {
    ws2812SetColor(i, 0, 0, 0);
  }

  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)led_buf, (BIT_OFFSET + BIT_WIDTH * led_cnt) * 1);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void ws2812SetColor(uint32_t index, uint8_t red, uint8_t green, uint8_t blue)
{
  uint8_t r_bit[8];
  uint8_t g_bit[8];
  uint8_t b_bit[8];
  uint8_t w_bit[8];

  uint32_t offset;


  for (int i=0; i<8; i++)
  {
    if (red & (1<<7))
    {
      r_bit[i] = BIT_HIGH;
    }
    else
    {
      r_bit[i] = BIT_LOW;
    }
    red <<= 1;

    if (green & (1<<7))
    {
      g_bit[i] = BIT_HIGH;
    }
    else
    {
      g_bit[i] = BIT_LOW;
    }
    green <<= 1;

    if (blue & (1<<7))
    {
      b_bit[i] = BIT_HIGH;
    }
    else
    {
      b_bit[i] = BIT_LOW;
    }
    blue <<= 1;

    w_bit[i] = BIT_LOW;
  }

  offset = BIT_OFFSET;

  memcpy(&led_buf[offset + index*BIT_WIDTH + 8*0], g_bit, 8*1);
  memcpy(&led_buf[offset + index*BIT_WIDTH + 8*1], r_bit, 8*1);
  memcpy(&led_buf[offset + index*BIT_WIDTH + 8*2], b_bit, 8*1);
  memcpy(&led_buf[offset + index*BIT_WIDTH + 8*3], w_bit, 8*1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
