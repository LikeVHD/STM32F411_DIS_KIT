// File nay bink blue LED trong board Discovery

#include "main.h"
#include <stdint.h>

__attribute__((section(".ver_number"))) uint32_t u32versionNumber = 10;

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

uint32_t sysTickCouter = 0;
void SysTick_Handler()
{
	sysTickCouter++;
}

//-------------------------------------------------------------
// KHOI TAO SYSTEM TICK
//-------------------------------------------------------------
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

int main()
{
	// Bao voi ARM da di chuyen bang vector table den dia chi 0x8004080;
	uint32_t* VTOR = (uint32_t*)0xE000ED08;
	*VTOR = 0x8004080;
	sysTick_Init();
	ledInit();

	uint32_t temp = u32versionNumber;
	(void)temp;

	while (1)
	{
		GPIOD_ODR |= LED_BLUE;		// On blue LED
		custom_Delay(500);
		GPIOD_ODR &= ~LED_BLUE;		// Off blue LED
		custom_Delay(500);
	}

	return 0;
}
