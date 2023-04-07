/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bootloader.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
   *
  ******************************************************************************
  */

#include "main.h"

int main(void)
{

	uint32_t *verAddr = 0;
	verAddr = 0x8004000;
	uint32_t firmwave01_ver = *verAddr;

	verAddr = 0x8008000;
	uint32_t firmwave02_ver = *verAddr;

	void (*firmwave_Reset_Handler)();
	uint32_t *resetHanderAddr = 0;

	if (firmwave01_ver > firmwave02_ver)
		resetHanderAddr = 0x8004084;
	else
		resetHanderAddr = 0x8008084;

	firmwave_Reset_Handler = (*resetHanderAddr|1);
	firmwave_Reset_Handler();

	while (1)
	{

	}
	return 0;
}
