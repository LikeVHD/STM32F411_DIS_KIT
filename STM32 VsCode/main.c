/*
 * main.c
 *
 *  Created on: Apr 24, 2023
 *      Author: Administrator
 */

#include <stdint.h>

//-------------------------------------------------------------
// DINH NGHIA CAC LED TREN CHAN PD12, PD13, PD14, PD15
//------------------------------------------------------------
#define LED_GREEN	(1U << 12)
#define LED_ORANGE	(1U << 13)
#define LED_RED		(1U << 14)
#define LED_BLUE	(1U << 15)

//-------------------------------------------------------------
// DINH NGHIA CAC THANH GHI PORT D
//-------------------------------------------------------------
#define GPIOD_MODER		(*((uint32_t*)(0x40020C00 | 0x00)))	// GPIO port D mode select register
#define GPIOD_OTYPER	(*((uint32_t*)(0x40020C00 | 0x04)))	// GPIO port D output type register
#define GPIOD_ODR		(*((uint32_t*)(0x40020C00 | 0x14)))	// GPIO port D output data register

uint32_t systick_cnt = 0;
void SysTick_Handler()
{
	systick_cnt++;
}
void system_tick_init()
{
	uint32_t* CSR = (uint32_t* )0xe000e010;
	uint32_t* RVR = (uint32_t* )0xe000e014;

	*RVR = 15999;
	*CSR |= (1<<1)|(1<<0)|(1<<2);
}

void SystemInit()
{
	
}

/**
 *@brief: Set clock 100MHz for system use PLL.
 */

/// @brief: Ham khoi tao clock cho he thong
void clock_Init()
{
	uint32_t *RCC_CR = (uint32_t*)0x40023800;
	uint32_t *RCC_PLLCFGR = (uint32_t*)0x40023804;
	uint32_t *RCC_CFGR = (uint32_t*)0x40023808;

	*RCC_CR |= 1U << 16U;			// HSE clock enable
	while (!((*RCC_CR >> 17) & 1));	// Wait until HSE clock ready flag

	*RCC_CR &= ~(1U << 24U);		// Disable before setup PLL
	while (((*RCC_CR >> 25) & 1));	// Wait PLL is unlocked


	*RCC_PLLCFGR |= (8U << 0);		// PLLM Division
	*RCC_PLLCFGR |= (200U << 6U);	// PLLN multiplication 200
	*RCC_PLLCFGR |= (0U << 16);		// PLLP Division 2
	*RCC_PLLCFGR |= (1U << 22U);	// HSE oscillator clock selected as PLL
	*RCC_PLLCFGR |= (5U << 24U);	// PLLQ division 5

	*RCC_CR |= 1U << 24U;			// PLL enable
	*RCC_CFGR |= 4U << 10U;			// Set APB Low speed pre-scaler (APB1) = 50MHz

	/*
		 * Set number of wait states according to CPU clock (HCLK) frequency
		 * Voltage range 2.7 V - 3.6 V -> 3 WS (4 CPU cycles) with 90 < HCLK ≤ 100
		 */
		uint32_t* FLASH_ACR = (uint32_t*)0x40023c00;
		*FLASH_ACR |= (3U << 0U);

		const unsigned char clock_sw = 0b10;	
		*RCC_CFGR |= clock_sw;					//select PLL as system clock
		while((*RCC_CFGR & clock_sw) != clock_sw);

		/*
		 * AHB = 100Mhz (max)
		 * APB1 = 50Mhz (max)
		 * APB2 = 100Mhz (max)
		 */
}

/// @brief: Ham khoi tao pin PD12 cho viec dieu khien LED
void led_Init()
{
	uint32_t *RCC_AHB1ENR = (uint32_t*)0x40023830;
	*RCC_AHB1ENR |= 1U << 3U;

	GPIOD_MODER |= (1U << 24U);			// Set PD12 as output
	GPIOD_OTYPER &= ~(1U << 12U);
}

/// @brief: Ham dieu khien LED
/// @param state 
void led_Control(char state)
{
	if(state)
		GPIOD_ODR |= 1U << 12U;
	else
		GPIOD_ODR &= ~(1U << 12U);
}

/// @brief: CHUONG TRINH CHINH
/// @param  NONE
/// @return 0
int main (void)
{
	clock_Init();
	led_Init();
	led_Control(0);
	while (1)
	{

	}

	return 0;
}
;