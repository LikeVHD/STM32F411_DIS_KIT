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
static void MX_GPIO_Init(void);
void i2c_Init(void);
void i2c_Write(uint8_t, uint8_t);
uint8_t i2c_Read(uint8_t);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void i2c_Init()
{
	__HAL_RCC_GPIOB_CLK_ENABLE();	// Cap clock cho port B

	uint32_t *GPIOB_MODER = (uint32_t*)(0x40020400);
	uint32_t *GPIOB_AFRL = (uint32_t*)(0x40020420);
	uint32_t *GPIOB_AFRH = (uint32_t*)(0x40020424);

	*GPIOB_MODER |= (2U << 12U) | (2U << 18U);	// Set pin 6 va pin 9 o chuc nang Alternate
	*GPIOB_AFRL |= (4U << 24U);					// Pin 6 o chuc nang I2C
	*GPIOB_AFRH |= (4U << 4U);					// Pin 9 o chuc nang I2C

	__HAL_RCC_I2C1_CLK_ENABLE();	// Cap clock cho bo i2c1
	uint32_t *I2C1_CR1 = (uint32_t*)(0x40005400);
	uint32_t *I2C1_CR2 = (uint32_t*)(0x40005404);
	uint32_t *I2C1_DR = (uint32_t*)(0x40005410);
	uint32_t *I2C1_SR1 = (uint32_t*)(0x40005414);
	uint32_t *I2C1_SR2 = (uint32_t*)(0x40005418);
	uint32_t *I2C1_CCR = (uint32_t*)(0x4000541c);

	*I2C1_CR1 &= ~(1U << 0);		// Disable i2c before setup i2c
	*I2C1_CR2 |= 16;				// Configure peripheral clock frequency at 16 MHz
	*I2C1_CCR = 40;					// Set clock 16 MHz / (40*2) = 200kHz

	*I2C1_CR1 |= 1U << 0;			// Enable i2c
}

void i2c_Write(uint8_t data, uint8_t reg_addr)
{
	uint32_t *I2C1_CR1 = (uint32_t*)(0x40005400);
	uint32_t *I2C1_DR = (uint32_t*)(0x40005410);
	uint32_t *I2C1_SR1 = (uint32_t*)(0x40005414);
	uint32_t *I2C1_SR2 = (uint32_t*)(0x40005418);
	uint32_t temp = 0;

	*I2C1_CR1 |= 1U << 8U;			// Generate a Start bit
	while (!((*I2C1_SR1 >> 0) & 1));	// Cho bit Start gui hoan thanh
	*I2C1_DR = 0x32;					// Gui dia chi
	while (!((*I2C1_SR1 >> 1) & 1));	// Cho gui hoan thanh dia chi
	temp = *I2C1_SR2;					// Clear ADDR

	*I2C1_DR = reg_addr; 				// Gui lenh ghi toi dia chi
	while (!((*I2C1_SR1 >> 2) & 1));	// Cho lenh duoc gui thanh cong
	*I2C1_DR = data; 					// Gui du lieu vao dia chi da dinh
	while (!((*I2C1_SR1 >> 2) & 1));	// Cho du lieu duoc gui thanh cong
	*I2C1_CR1 |= 1U << 9U;				// Generate a Stop bit
}

uint8_t i2c_Read(uint8_t reg_addr)
{
	uint32_t *I2C1_CR1 = (uint32_t*)(0x40005400);
	uint32_t *I2C1_DR = (uint32_t*)(0x40005410);
	uint32_t *I2C1_SR1 = (uint32_t*)(0x40005414);
	uint32_t *I2C1_SR2 = (uint32_t*)(0x40005418);
	uint32_t temp = 0;

	*I2C1_CR1 |= 1U << 8U;				// Gui Start bit
	while (!((*I2C1_SR1 >> 0) & 1));	// Cho bit Start gui hoan thanh
	*I2C1_DR = 0x32;					// Gui dia chi
	while (!((*I2C1_SR1 >> 1) & 1));	// Cho gui hoan thanh dia chi
	temp = *I2C1_SR2;					// Clear ADDR
	*I2C1_DR = reg_addr; 				// Gui lenh doc
	while (!((*I2C1_SR1 >> 2) & 1));	// Cho lenh doc duoc gui thanh cong

	*I2C1_CR1 |= 1U << 8U;				// Generate a Start bit
	while (!((*I2C1_SR1 >> 0) & 1));
	*I2C1_DR = 0x33;					// Gui lenh doc toi slave
	while (!((*I2C1_SR1 >> 1) & 1));	// Check ADDR is send
	temp = *I2C1_SR2;					// Clear ADDR
	while (!((*I2C1_SR1 >> 6) & 1));	// Cho nhan du lieu hoan thanh
	temp = *I2C1_DR;					// Data duoc luu trong temp

	*I2C1_CR1 |= 1U << 9U;				// Generate a Stop bit
	return temp;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint8_t i2cReadData = 0;

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
  i2c_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  i2cReadData = i2c_Read(0x20);
	  i2c_Write(0x0F, 0x20);
	  i2cReadData = i2c_Read(0x20);

while (1)
  {
    /* USER CODE END WHILE */
	  HAL_Delay(1000);


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
