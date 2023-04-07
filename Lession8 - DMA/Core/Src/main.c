/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DMA1BASE 		0x40026000
#define S5CR	 		(*((uint32_t*)(DMA1BASE + 0x10 + 0x18 * 5))) // Stream 5 Control register
#define S5NDTR	 		(*((uint32_t*)(DMA1BASE + 0x14 + 0x18 * 5))) // DMA stream 5 number of data register
#define S5PAR	 		(*((uint32_t*)(DMA1BASE + 0x18 + 0x18 * 5))) // Stream 5 Control register
#define S5M0AR	 		(*((uint32_t*)(DMA1BASE + 0x1C + 0x18 * 5))) // Stream 5 Control register

#define BUFFER_SIZE		32

#define GPIOA_MODER		(*((uint32_t*)(0x40020000 | 0x00)))	// GPIO port A mode select register
#define GPIOA_AFRL		(*((uint32_t*)(0x40020000 | 0x20)))	// GPIO alternate function low register
#define CR1				(*((uint32_t*)(0x4000440C)))	//
#define CR3				(*((uint32_t*)(0x40004414)))	//

#define BRR				(*((uint32_t*)(0x40004408)))	//
#define DR				(*((uint32_t*)(0x40004404)))	//
#define SR				(*((uint32_t*)(0x40004400)))	///* USER CODE END PTD *//* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Uart2_Rx_Buffer[BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// initialize DMA
void DMA_Init()
{
	__HAL_RCC_DMA1_CLK_ENABLE();

	S5NDTR = sizeof(Uart2_Rx_Buffer);
	S5PAR = 0x40004404;					// Dia chi nguon (dia chi thanh ghi DR cua uart2)
	S5M0AR = (uint32_t)Uart2_Rx_Buffer;	// Dia chi dich	(dia chi buffer tren Ram)
	S5CR |= (0b100 << 25) | (1U << 10); // Chon chanel 4 + tu dong tang dia chi.
	S5CR |= 1U;							// Stream 5 Enable.
}

void uart2_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();		// Enable clock for GPIOD
	GPIOA_MODER |= (2U << 4) | (2U << 6); // PA2, PA3 as Alternate function mode
	GPIOA_AFRL &= ~(0xFFU << 8);
	GPIOA_AFRL |= (7U << 8) | (7U << 12);

	__HAL_RCC_USART2_CLK_ENABLE();
	BRR = (104U << 4) | (3U << 0);
	CR1 |= (1U << 3) | (1U << 2) | (1U << 13);

	CR3 |= (1U << 6);


//	CR1 |= (1U << 5);			// Enable RXNEIE bit to general interrupt
//	NVIC_ISER1 |= (1U << 6);	// Enable USART2 reception
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
  /* USER CODE BEGIN 2 */
  uart2_init();
  DMA_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
