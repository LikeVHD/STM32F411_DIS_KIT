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

//-------------------------------------------------------------
// DINH NGHIA CAC LED TREN CHAN PD12, PD13, PD14, PD15
//------------------------------------------------------------
#define LED_GREEN	(1U << 12)
#define LED_ORANGE	(1U << 13)
#define LED_RED		(1U << 14)
#define LED_BLUE	(1U << 15)

//-------------------------------------------------------------
// DINH NGHIA NUT BAM TREN PIN PA0
//-------------------------------------------------------------
#define BUTTON		(*((uint32_t*)(0x40020000 + 0x10)) & 1U)

//-------------------------------------------------------------
// DINH NGHIA CAC THANH GHI PORT D
//-------------------------------------------------------------
#define GPIOD_MODER		(*((uint32_t*)(0x40020C00 | 0x00)))	// GPIO port D mode select register
#define GPIOD_OTYPER	(*((uint32_t*)(0x40020C00 | 0x04)))	// GPIO port D output type register
#define GPIOD_ODR		(*((uint32_t*)(0x40020C00 | 0x14)))	// GPIO port D output data register

//-------------------------------------------------------------
// DINH NGHIA CAC THANH GHI PORT A
//-------------------------------------------------------------
#define GPIOA_MODER		(*((uint32_t*)(0x40020000 | 0x00)))	// GPIO port A mode select register

// For I/O function
#define GPIOA_IDR		(*((uint32_t*)(0x40020000 | 0x10)))	// GPIO port A input data register
#define GPIOA_PUPDR		*((uint32_t*)(0x40020000 | 0x0C))	// GPIO port A pull-up/pull-down register

// For alternate function
#define GPIOA_AFRL		(*((uint32_t*)(0x40020000 | 0x20)))	// GPIO alternate function low register
#define CR1				(*((uint32_t*)(0x4000440C)))		//
#define BRR				(*((uint32_t*)(0x40004408)))		//

// Define the USART2 registers
#define USART2_BASE		(*((uint32_t*)(0x40004400)))
#define USART2_DR		(*((uint32_t*)(USART2_BASE + 0x04)))//
#define USART2_SR		(*((uint32_t*)(USART2_BASE + 0x00)))//
//-------------------------------------------------------------
// DINH NGHIA CAC THANH NGAT NGOAI
//-------------------------------------------------------------
#define EXTI_IMR		(*((uint32_t*)(0x40013C00 | 0x00)))	// Interrupt mask register
#define EXTI_RTSR		(*((uint32_t*)(0x40013C00 | 0x08)))	// Rising trigger selection register
#define EXTI_PR			(*((uint32_t*)(0x40013C00 | 0x14)))	// Pending register


#define NVIC_ISER0		(*((uint32_t*)(0xE000E100 | 0x00)))	// Interrupt set-enable register 0
#define VTOR			(*((uint32_t*)(0xE000ED08 | 0x00)))

//-------------------------------------------------------------
// DINH NGHIA CAC DIA CHI CO BAN CUA SECTOR DE XOA
//-------------------------------------------------------------
#define FLASH_CR_PGBit	 			1U	// PG (Program) bit
#define FLASH_CR_SERBit 			2U	// SER (Sector Erase) bit
#define FLASH_SECTOR_ADDRESS	((uint32_t)0x08004000)

#define	FLASH_CR 		(*((uint32_t*)0x40023c10))	// Flash control register
#define	FLASH_SR 		(*((uint32_t*)0x40023c0c))	// Flash status register
#define	FLASH_KEYR 		(*((uint32_t*)0x40023c04))	// Flash key register

//-------------------------------------------------------------
// DINH NGHIA CAC DIA CHI CO BAN CUA DMA
//-------------------------------------------------------------
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
#define SR				(*((uint32_t*)(0x40004400)))	//
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void ledInit(void);
//void ledCtrl(LED_NUMBER_t ledNumber, LED_STATE_t ledState);
void buttonInit(void);
void exti0Init(void);
void ISR_EXTI0(void);
void uart2_init(void);
void uartSend1Byte(uint8_t);
void uartRecvByte(char*, int);
void Custom_Flash_Erase_Sector(uint8_t);
void Custom_Flash_Program(uint32_t, uint8_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//-------------------------------------------------------------
// DELAY WITH SYSTEM TICK
//-------------------------------------------------------------

uint32_t sysTickCouter = 0;
void SysTick_Handler()
{
	sysTickCouter++;
}

//---------------------
// KHOI TAO SYSTEM TICK
//---------------------
void sysTick_Init()
{
	uint32_t *CSR = (uint32_t*)0xe000e010;
	uint32_t *RVR = (uint32_t*)0xe000e014;

	*RVR = 15999;
	*CSR |= (1U << 1)|(1U << 0)|(1U << 2);
}

void custom_Delay(uint32_t mSec)
{
	sysTickCouter = 0;
	while (sysTickCouter < mSec);
}


//-------------------------------------------------------------
// KHOI TAO PD12, PD13, PD14, PD15 in OUTPUT MODE (push-pull)
//-------------------------------------------------------------
void ledInit()
{
	__HAL_RCC_GPIOD_CLK_ENABLE();		// Enable clock for GPIOD
	/*
	 * to do:
	 * set PD12, PD13, PD14, PD15 in OUTPUT MODE (push-pull)
	 */
	GPIOD_MODER &= ~(0xFF << 24);		// Clear
	GPIOD_MODER |= (0b01 << 24) | (0b01 << 26) | (0b01 << 28) | (0b01 << 30);

	GPIOD_OTYPER &= ~(0b1111 << 12);
}

//void ledCtrl(LED_NUMBER_t ledNumber, LED_STATE_t ledState)
//{
////	uint32_t *GPIOD_ODR	= (uint32_t*)(0x40020C00 + 0x14);
//	if(ledState == LED_ON)
//	{
//		GPIOD_ODR |= LED_GREEN << ledNumber;
//	}
//	else
//	{
//		GPIOD_ODR &= ~(LED_GREEN << ledNumber);
//	}
//}

void DMA_Init()
{
	__HAL_RCC_DMA1_CLK_ENABLE();

	S5NDTR = sizeof(Uart2_Rx_Buffer);
	S5PAR = 0x40004404;					// Dia chi nguon (dia chi thanh ghi DR cua uart2)
	S5M0AR = (uint32_t)Uart2_Rx_Buffer;	// Dia chi dich	(dia chi buffer tren Ram)
	S5CR |= (0b100 << 25) | (1U << 10); // Chon chanel 4 + tu dong tang dia chi.
	S5CR |= 1U;							// Stream 5 Enable.
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


//-------------------------------------------------------------
// KHOI TAO NUT BAM TREN PIN PA0 (PA0 as INPUT)
//-------------------------------------------------------------

void buttonInit()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();	// Enable clock for GPIOA
	/*
	 * to do:
	 * set PA0 is input (no pull up no pull down)
	 */
	GPIOA_MODER &= ~(3U);		// PA0 as input
	GPIOA_PUPDR &= ~(3U);		// No pull up, no pull down
}

//-------------------------------------------------------------
// KHOI TAO NGAT NGOAI 0 (EXTI0)
//-------------------------------------------------------------
void exti0Init()
{
	EXTI_IMR |= 1U << 0;
	EXTI_RTSR |= 1U << 0;
	NVIC_ISER0 |= 1U << 6;
}

//-------------------------------------------------------------
// TRINH PHUC VU NGAT NGOAI 0
//-------------------------------------------------------------
void ISR_EXTI0()
{
//	ledCtrl(LED3, LED_ON);
//	ledCtrl(LED1, LED_ON);
	EXTI_PR |= 1U << 0;		// Clear interrupt flag
}

//-------------------------------------------------------------
// UART2 INIT
//-------------------------------------------------------------
/*
 * Cac viec can lam de khoi tao UART2:
 * 1. Chon che do alternate cua chan GPIO.
 * 2. Chon baudrate.
 * 3. Khoi tao frame, stop bit, parity bit.
 * 4. Chon che do (Tx hay Rx hay ca 2).
 */
void uart2_init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();		// Enable clock for GPIOD
	GPIOA_MODER |= (2U << 4) | (2U << 6); // PA2, PA3 as Alternate function mode
	GPIOA_AFRL &= ~(0xFFU << 8);
	GPIOA_AFRL |= (7U << 8) | (7U << 12);

	__HAL_RCC_USART2_CLK_ENABLE();
	BRR = (104U << 4) | (3U << 0);
	CR1 = (1U << 3) | (1U << 2) | (1U << 13);
}

//-------------------------------------------------------------
// UART SEND 1 BYTE
//-------------------------------------------------------------
void uartSend1Byte(uint8_t data)
{
	while (((USART2_SR >> 7)&1) != 1);
	USART2_DR = data;
	while (((USART2_SR >> 6)&1) != 1);
}

//-------------------------------------------------------------
// UART RECIVE DATA
//-------------------------------------------------------------
/*
 * Truyen vao dia chi nhan du lieu va kich thuoc du lieu muon nhan
 */
void uartRecvByte(char *buff, int size)
{
	/*
	 * To do:
	 * 1. Read bit RXNE (bit 5 in Status register (USART_SR)).
	 *
	 */

	for (int i = 0; i < size; i++)
	{
		while(((USART2_SR >> 5) & 1) == 0); // Cho o day khi nhan du du lieu (bit RXNE duoc set len 1)
			buff[i] = DR;
	}
}

void Custom_Flash_Erase_Sector(uint8_t u8SectorNumber)
{
	if ((FLASH_CR_LOCK >> 31) & 1)		// Check LOCK bit
	{
		/* Unlock the flash interface */
		FLASH_KEYR = 0x45670123;
		FLASH_KEYR = 0xCDEF89AB;
	}
	// 1.
	while ((FLASH_SR_BSY >> 16) & 1);		// Wait until BSY is clean
	// 2.
	FLASH_CR |= FLASH_CR_SER;				// Sector erase activated
	FLASH_CR |= (u8SectorNumber << 3);	// "u8SectorNumber" Erase
	// 3.
	FLASH_CR |= FLASH_CR_STRT;				// Start erase bit
	// 4.
	while ((FLASH_SR_BSY >> 16) & 1);		// Wait until BSY is clean
	FLASH_CR &= ~FLASH_CR_SER;				// Sector erase deactivated
}

void Custom_Flash_Program(uint32_t u32Addr, uint8_t u8Data)
{
	if ((FLASH_CR_LOCK >> 31) & 1)		// Kiem tra bit LOCK
	{
		// Unlock Flash
		FLASH_KEYR = 0x45670123;
		FLASH_KEYR = 0xCDEF89AB;
	}
	// 1.
	while ((FLASH_SR_BSY >> 16) & 1);		// Wait until BSY is clean
	// 2.
	FLASH_CR |= FLASH_CR_PG;				// Flash programming activated
	// 3.
	uint8_t *pu8Temp = (uint8_t*)u32Addr;
	*pu8Temp = u8Data;
	// 4.
	while ((FLASH_SR_BSY >> 16) & 1);		// Wait until BSY is clean
	// 2.
	FLASH_CR &= ~FLASH_CR_PG;				// Flash programming deactivated
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//-------------------------------------------------------------
	// DI CHUYEN BANG VECTOR NGAT (MOVE VECTOR TABLE)
	//-------------------------------------------------------------
	// Move vector table to RAM (0x20000000)
	memcpy(0x20000000, 0x00, 0x198);

	// Set NVIC vector table offset to 0x20000000
	VTOR = 0x2000000;

	// Register ISR_EXTI0 to EXTI0 interrup event
	uint32_t *temp = 0x20000058;
	*temp = (uint32_t)ISR_EXTI0 | 1U;
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
