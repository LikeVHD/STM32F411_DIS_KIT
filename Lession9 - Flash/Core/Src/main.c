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


/* Private typedef -----------------------------------------------------------*/
//-------------------------------------------------------------
// DINH NGHIA CAC THANH GHI PORT A
//-------------------------------------------------------------
#define GPIOA_MODER		(*((uint32_t*)(0x40020000 + 0x00)))	// GPIO port A mode select register

// For I/O function
#define GPIOA_IDR		(*((uint32_t*)(0x40020000 + 0x10)))	// GPIO port A input data register
#define GPIOA_PUPDR		*((uint32_t*)(0x40020000 + 0x0C))	// GPIO port A pull-up/pull-down register

// For alternate function
#define GPIOA_AFRL		(*((uint32_t*)(0x40020000 + 0x20)))	// GPIO alternate function low register
#define CR1				(*((uint32_t*)(0x4000440C)))		//
#define CR3				(*((uint32_t*)(0x40004414)))		//
#define BRR				(*((uint32_t*)(0x40004408)))		//

/* Define the base address of the sector to erase */
#define FLASH_CR_PGBit	 			1U	// PG (Program) bit
#define FLASH_CR_SERBit 			2U	// SER (Sector Erase) bit
#define FLASH_SECTOR_ADDRESS	((uint32_t)0x08004000)

#define	FLASH_CR 		(*((uint32_t*)0x40023c10))	// Flash control register
#define	FLASH_SR 		(*((uint32_t*)0x40023c0c))	// Flash status register
#define	FLASH_KEYR 		(*((uint32_t*)0x40023c04))	// Flash key register

#define	SR 				(*((uint32_t*)0x40004400))	// Status register
#define	DR 				(*((uint32_t*)0x40004404))	// Data register

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
char rx_buf[5821];
uint8_t rx_recv_done_flag = 0;
int index = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void Custom_Flash_Erase_Sector(uint8_t);
void Custom_Flash_Program(uint32_t, uint8_t*, uint32_t);

void uart2_init(void);
void UART_Send_1Byte(char);
void UART_Send_String(char*);
void UART_Recv_Bytes(char*, int);

/* Private user code ---------------------------------------------------------*/
__attribute__((section( ".funInRam" ))) void Custom_Flash_Erase_Sector(uint8_t u8SectorNumber)
{
	if ((FLASH_CR >> 31) & 1)		// Check LOCK bit
	{
		/* Unlock the flash interface */
		FLASH_KEYR = 0x45670123;
		FLASH_KEYR = 0xCDEF89AB;
	}
	// 1.
	while ((FLASH_SR >> 16) & 1);		// Wait until BSY is clean
	// 2.
	FLASH_CR |= FLASH_CR_SER;				// Sector erase activated
	FLASH_CR |= (u8SectorNumber << 3);		// "u8SectorNumber" Erase
	// 3.
	FLASH_CR |= FLASH_CR_STRT;				// Start erase bit
	// 4.
	while ((FLASH_SR >> 16) & 1);		// Wait until BSY is clean
	FLASH_CR &= ~FLASH_CR_SER;				// Sector erase deactivated
}

__attribute__((section( ".funInRam" ))) void Custom_Flash_Program(uint32_t u32Addr, uint8_t* u8Data, uint32_t size)
{
	// 1.
	if ((FLASH_CR >> 31) & 1)		// Kiem tra bit LOCK
	{
		// Unlock Flash
		FLASH_KEYR = 0x45670123;
		FLASH_KEYR = 0xCDEF89AB;
	}

	// 2.
	while ((FLASH_SR >> 16) & 1);		// Wait until BSY is clean
	FLASH_CR |= FLASH_CR_PG;				// Flash programming activated

	// 3.
	uint8_t *pu8Temp = (uint8_t*)u32Addr;
	for (int i = 0; i < size; i++)
	{
		*pu8Temp = u8Data[i];
		pu8Temp++;
	}

	// 4.
	while ((FLASH_SR >> 16) & 1);		// Wait until BSY is clean
	FLASH_CR &= ~FLASH_CR_PG;				// Flash programming deactivated
}

void uart2_init()
{
	// set PA2 (U2Tx) PA3 (U2Rx)
	__HAL_RCC_GPIOA_CLK_ENABLE();		// Enable clock for GPIOD
	GPIOA_MODER &= ~(0x0F << 4);
	GPIOA_MODER |= (2U << 4) | (2U << 6); // PA2, PA3 as Alternate function mode

	GPIOA_AFRL &= ~(0xFFU << 8);
	GPIOA_AFRL |= (7U << 8) | (7U << 12);

	__HAL_RCC_USART2_CLK_ENABLE();
	BRR = (104U << 4) | (3U << 0);		// Set baudrate 9600
	CR1 = (1U << 3) | (1U << 2) | (1U << 13);

	CR3 = (1U<<6);	// enable DMA
}


void UART_Send_1Byte(char data)
{
	while (((SR >> 7) &1) != 1);
	DR = data;
	while (((SR >> 6) &1) != 1);
}

void UART_Send_String(char* str)
{
	uint32_t index = 0;
	while (str[index] != 0)
	{
		UART_Send_1Byte(str[index]);
		index++;
	}
}

void UART_Recv_Bytes(char* buff, int size)
{
	for(int i = 0; i < size; i++)
	{
		while(((SR >> 5)&1) == 0);
		buff[i] = DR;
	}
}


void USART2_IRQHandler()
{
	rx_buf[index++] = DR;
	if(index >= 31)
		index = 0;
}

void DMA_init()
{
	__HAL_RCC_DMA1_CLK_ENABLE();
	uint32_t* S5CR = (uint32_t*)0x40026088;
	uint32_t* S5NDTR = (uint32_t*)0x4002608c;
	uint32_t* S5PAR = (uint32_t*)0x40026090;
	uint32_t* S5M0AR = (uint32_t*)0x40026094;

	*S5NDTR = sizeof(rx_buf);
	*S5PAR = 0x40004404;
	*S5M0AR = rx_buf;
	*S5CR |= (0b100<<25) | (1<<10) | (1<<4);
	*S5CR |= 1;

	uint32_t* ISER0 = (uint32_t*)0xe000e100;
	*ISER0 |= (1<<16);
}

void DMA1_Stream5_IRQHandler()
{
	rx_recv_done_flag = 1;
	__asm("NOP");
	uint32_t* HIFCR = (uint32_t*)0x4002600c;
	*HIFCR |= (1<<11);
}

__attribute__((section( ".funInRam" )))void update_Firmwave()
{
	// 1. Tat cac System tick
	uint32_t* SYSTICK_CSR = (uint32_t*)0xe000e010;
	*SYSTICK_CSR &= ~(1<<0);

	// 2. Thuc hien ghi file vao Flash
	Custom_Flash_Erase_Sector(0);
	Custom_Flash_Program(0x08000000, (uint8_t*)rx_buf, sizeof(rx_buf));

	// 3. Tao ngat tu dong reset he thong
	uint32_t* AIRCR = (uint32_t*)0xe000ed0c;
	*AIRCR = (0x5fa<<16) | (1U<<2);
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  uart2_init();
  DMA_init();

  char msg[] = "This is firmwave to update firmwave\r\n";
  UART_Send_String(msg);

  while (1)
  {
	  __asm("NOP");
	  if(rx_recv_done_flag == 1)
		  update_Firmwave();
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
//	  HAL_Delay(500);
  }
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
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

	  /*Configure GPIO pin : PA0 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
	  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
