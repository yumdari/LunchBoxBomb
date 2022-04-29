/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define NEO_LED_CNT   8
#define BIT_WIDTH     32
#define BIT_OFFSET    66
#define BIT_LOW       25
#define BIT_HIGH      47


uint8_t neo_data[BIT_OFFSET + BIT_WIDTH*NEO_LED_CNT];


void neoSetPixel(uint8_t ch, uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t r_bit[8];
  uint8_t g_bit[8];
  uint8_t b_bit[8];
  uint8_t w_bit[8];


  for (int i=0; i<8; i++)
  {
    if (r & (1<<7)) r_bit[i] = BIT_HIGH;
    else            r_bit[i] = BIT_LOW;
    r <<= 1;

    if (g & (1<<7)) g_bit[i] = BIT_HIGH;
    else            g_bit[i] = BIT_LOW;
    g <<= 1;

    if (b & (1<<7)) b_bit[i] = BIT_HIGH;
    else            b_bit[i] = BIT_LOW;
    b <<= 1;

    w_bit[i] = BIT_LOW;

    neo_data[BIT_OFFSET + BIT_WIDTH*ch + 8*0 + i] = g_bit[i];
    neo_data[BIT_OFFSET + BIT_WIDTH*ch + 8*1 + i] = r_bit[i];
    neo_data[BIT_OFFSET + BIT_WIDTH*ch + 8*2 + i] = b_bit[i];
    neo_data[BIT_OFFSET + BIT_WIDTH*ch + 8*3 + i] = w_bit[i];
  }
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
  MX_DMA_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);


  // 80us low
  for (int i=0; i<BIT_OFFSET; i++)
  {
    neo_data[i] = 0;
  }

  for (int i=0; i<NEO_LED_CNT; i++)
  {
    for (int j=0; j<BIT_WIDTH; j++)
    {
      neo_data[BIT_OFFSET + BIT_WIDTH*i + j] = BIT_LOW;
    }
  }

  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)neo_data, BIT_OFFSET + BIT_WIDTH*NEO_LED_CNT);


  for (int i=0; i<NEO_LED_CNT; i++)
  {
    neoSetPixel(i, 0, 0, 50);
  }

  uint32_t led_index = 0;
  uint32_t led_color = 0;
  uint32_t pre_time;
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

      for (int i=0; i<NEO_LED_CNT; i++)
      {
        if (i == led_index)
        {
          switch(led_color)
          {
            case 0:
              neoSetPixel(i, 50, 0, 0);
              break;
            case 1:
              neoSetPixel(i,  0, 50, 0);
              break;
            case 2:
              neoSetPixel(i, 0, 0, 50);
              break;
          }
        }
        else
        {
          neoSetPixel(i, 0, 0, 0);
        }
      }

      led_index = (led_index + 1)%8;

      if (led_index == 0)
      {
        led_color = (led_color + 1)%3;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
