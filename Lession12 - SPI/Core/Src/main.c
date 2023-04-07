/*
 * main.c
 *
 *  Created on: Mar 27, 2023
 *      Author: Administrator
 */

//---------------------------
// INCLUDE
//---------------------------

#include "main.h"
//#include <stdint.h>
//#include <stm32f411xe.h>


//---------------------------
// DEFINE
//---------------------------

#define GPIOA_MODER			(*((uint32_t*)(0x40020000)))	// GPIO port A mode select register
#define GPIOA_AFRL			(*((uint32_t*)(0x40020020)))	// GPIO A alternate function low register

#define GPIOE_MODER			(*((uint32_t*)(0x40021000)))	// GPIO port E mode select register
#define GPIOE_ODR			(*((uint32_t*)(0x40021014)))	// GPIO port E output data register
#define SENSOR_ACTIVE		GPIOE_ODR &= ~(1U << 3U)		// Sensor active at low level
#define SENSOR_INACTIVE		GPIOE_ODR |= (1U << 3U)

#define SPI1_CR1			(*((uint32_t*)(0x40013000)))	// SPI control register 1
#define SPI1_SR				(*((uint32_t*)(0x40013008)))	// SPI status register
#define SPI1_DR				(*((uint32_t*)(0x4001300c)))	// SPI data register
#define SPI_BSY_FLAG		((SPI1_SR >> 7) & 1)			// Busy flag = 0 (not busy)
#define SPI_TXE_FLAG		((SPI1_SR >> 1) & 1)			// Tx empty flag = 1 (empty)
#define SPI_RXNE_FLAG		((SPI1_SR >> 0) & 1)			// Rx not empty flag = 1 (not empty)

//---------------------------
// DECLARE VARIABLE
//---------------------------
uint32_t sysTickCouter = 0;
uint8_t id;
uint16_t x_Axis;
uint16_t y_Axis;
uint16_t z_Axis;

//---------------------------
// PROTOTYPE
//---------------------------

void SPI_Init(void);
void Custom_SPI_Init(void);
uint8_t SPI_Transmit(uint8_t);
uint8_t Custom_SPI_Sensor_Read(uint8_t);
void Custom_SPI_Sensor_Write(uint8_t, uint8_t);
void sysTick_Init(void);
void custom_Delay(uint32_t);
void L3GD20_Init(void);

//---------------------------
// DEFINITIONS OF FUNCTIONS
//---------------------------
void SysTick_Handler()
{
	sysTickCouter++;
}

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

void Custom_SPI_Init()
{
	//--------------
	// Cau hinh GPIO
	//--------------

	__HAL_RCC_GPIOA_CLK_ENABLE();	// Cap xung clock cho GPIO cong A
	// Chon chuc nang chuyen tiep cha chan 5, 6, 7 cua cong A
	GPIOA_MODER &= ~((3U << 10U) | (3U << 12U) | (3U << 14U));	// Reset mode at pin 5, 6, 7
	GPIOA_MODER |= ((2U << 10U) | (2U << 12U) | (2U << 14U));
	// Cau hinh chan GPIO 5, 6 va 7 tren Port A voi gia tri chuc nang chuyen tiep chan 5 (SPI 1).
	GPIOA_AFRL |= (0x555 << 20);

	__HAL_RCC_GPIOE_CLK_ENABLE();	// Cap xung clock cho GPIO cong E
	GPIOE_MODER &= ~(3U << 6U);
	GPIOE_MODER |= (1U << 6U);

	SENSOR_INACTIVE;

	//---------------
	// Khoi tao SPI
	//---------------

	// 1. Bat clock cho SPI 1.
	__HAL_RCC_SPI1_CLK_ENABLE();	// Cap xung clock cho bo SPI 1.

	// 2. Thiet lap toc do truyen du lieu.
	SPI1_CR1 |= (4U << 3U);			//100: fPCLK/32

	// 3. Thiet lap module SPI1 lam chu (Set bit MSTR trong thanh ghi CR1).
	SPI1_CR1 |= (1U << 2U);

	// 4. Thiet lap su dung che do dieu khien slave (software slave management mode).
    //	  Set bit SSM (Software Slave Management), bit SSI (Internal Slave Select) và bit SPE (SPI Enable) trong thanh ghi CR1.
	SPI1_CR1 |= (1U << 9U) | (1U << 8U) | (1U << 6);

}

uint8_t Custom_SPI_Sensor_Read(uint8_t cmd)
{
	SENSOR_ACTIVE;						// Active Sensor

	while (SPI_BSY_FLAG);				// Cho o day den duong truyen ranh
	uint8_t dataSend = cmd | (1 << 7);	// Chuan bi lenh doc sensor
	SPI1_DR = dataSend;					// Gui lenh doc
	while (!SPI_TXE_FLAG);				// Cho o day den khi gui xong lenh
	while (!SPI_RXNE_FLAG);				// Cho den khi Rx buff co du lieu
	while (SPI_BSY_FLAG);				// Cho den khi nhan xong du lieu

	uint32_t tempData = SPI1_DR;		// Doc du lieu de Clear spam data (sau lenh nay RXNE bit = 0)

	while (SPI_BSY_FLAG);				// Cho duong truyen ranh
	SPI1_DR = 0x00;						// Gui du lieu gia de tao ra clock
	while (!SPI_TXE_FLAG);				// Cho gui xong (Tx empty)
	while (!SPI_RXNE_FLAG);				// Cho nhan du lieu
	while (SPI_BSY_FLAG);				// Cho duong truyen ranh (nhan xong)

	tempData = SPI1_DR;					// Doc du lieu
	SENSOR_INACTIVE;
	return tempData;
}

void Custom_SPI_Sensor_Write(uint8_t data, uint8_t addr)
{
	SENSOR_ACTIVE;						// Active Sensor

	while (SPI_BSY_FLAG);				// Cho o day den duong truyen ranh
	uint8_t dataSend = addr;			// Chuan bi lenh ghi sensor
	SPI1_DR = dataSend;					// Gui dia chi
	while (!SPI_TXE_FLAG);				// Cho o day den khi gui xong lenh
	while (!SPI_RXNE_FLAG);				// Cho den khi Rx buff co du lieu
	while (SPI_BSY_FLAG);				// Cho den khi nhan xong du lieu

	uint32_t tempData = SPI1_DR;		// Doc du lieu de Clear spam data (sau lenh nay RXNE bit = 0)

	while (SPI_BSY_FLAG);				// Cho o day den duong truyen ranh
	dataSend = data;					// Chuan bi lenh ghi sensor
	SPI1_DR = dataSend;					// Gui data
	while (!SPI_TXE_FLAG);				// Cho o day den khi gui xong lenh
	while (!SPI_RXNE_FLAG);				// Cho den khi Rx buff co du lieu
	while (SPI_BSY_FLAG);				// Cho den khi nhan xong du lieu

	tempData = SPI1_DR;					// Doc du lieu de Clear spam data (sau lenh nay RXNE bit = 0)

	SENSOR_INACTIVE;
}

//---------------------------
// MAIN PROGRAM
//---------------------------

int main(void)
{
	sysTick_Init();
	Custom_SPI_Init();
	uint8_t index = Custom_SPI_Sensor_Read(0x20);
	Custom_SPI_Sensor_Write(0x0F, 0x20);
	index = Custom_SPI_Sensor_Read(0x20);

//	id = Custom_SPI_Sensor_Read(0x20);

	while (1)
	{
		uint8_t x_Low = Custom_SPI_Sensor_Read(0x28);
		uint8_t x_High = Custom_SPI_Sensor_Read(0x29);
		uint8_t y_Low = Custom_SPI_Sensor_Read(0x2A);
		uint8_t y_High = Custom_SPI_Sensor_Read(0x2B);
		uint8_t z_Low = Custom_SPI_Sensor_Read(0x2C);
		uint8_t z_High = Custom_SPI_Sensor_Read(0x2D);

		x_Axis = x_Low | (x_High << 8);
		y_Axis = y_Low | (y_High << 8);
		z_Axis = z_Low | (z_High << 8);
		custom_Delay(500);
//		SENSOR_INACTIVE;
//		test = GPIOE_ODR;
//		custom_Delay(500);
	}

	return 0;
}

//void SPI_Init(void)
//{
//    // Khởi tạo GPIO
//	//--------------------------------
//
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA clock enable
//    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
//
//    // Cấu hình chế độ của các chân GPIO 5, 6, và 7 trên Port A là đầu ra với chế độ tốc độ cao.
//    GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
//
//    // cấu hình chân GPIO 5, 6 và 7 trên Port A của vi điều khiển STM32F411 để sử dụng chức năng chuyển tiếp chân,
//    // với giá trị chức năng chuyển tiếp chân là 5.
//    GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);
//
//    // Khởi tạo SPI
//    //--------------------------------
//
//    // 1.	Bật clock cho SPI1 bằng cách set bit RCC_APB2ENR_SPI1EN trong thanh ghi RCC_APB2ENR lên 1.
//    	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
//
//    // 2.	Thiết lập tốc độ truyền dữ liệu của module SPI1 là tốc độ tối đa (không chia tốc độ),
//    //		bằng cách xóa bit BR (baud rate) trong thanh ghi CR1.
//		SPI1->CR1 &= ~SPI_CR1_BR;
//
//    // 3.	Thiết lập tốc độ truyền dữ liệu của module SPI1 bằng cách set bit BR_0 và BR_1 trong thanh ghi CR1.
//    // 		Tốc độ truyền dữ liệu được thiết lập là PCLK/8 (với PCLK là tần số clock của STM32F411).
//		SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1;
//
//    // 4. 	Thiết lập chế độ của clock tín hiệu là mức thấp ở trạng thái nghỉ.
//    //		Xóa bit CPOL (Clock Polarity) trong thanh ghi CR1.
//		SPI1->CR1 &= ~SPI_CR1_CPOL;
//
//    // 5.	Thiết lập chế độ truyền dữ liệu lấy mẫu trên cạnh thứ nhất của clock tín hiệu.
//    //		Xóa bit CPHA (Clock Phase) trong thanh ghi CR1.
//		SPI1->CR1 &= ~SPI_CR1_CPHA;
//
//    // 6.	Thiết lập module SPI1 làm chủ.
//    //		Set bit MSTR (Master Selection) trong thanh ghi CR1.
//		SPI1->CR1 |= SPI_CR1_MSTR;
//
//    // 7.	Thiết lập module SPI1 sử dụng chế độ điều khiển slave (software slave management mode).
//    //		Set bit SSM (Software Slave Management), bit SSI (Internal Slave Select) và bit SPE (SPI Enable) trong thanh ghi CR1.
//		SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE;
//}


//
//uint8_t SPI_Transmit(uint8_t data)
//{
//    while(!(SPI1->SR & SPI_SR_TXE));
//    SPI1->DR = data;
//    while(!(SPI1->SR & SPI_SR_RXNE));
//    return SPI1->DR;
//}


//void L3GD20_Init(void)
//{
//    // Cấu hình thanh ghi CTRL_REG1
//    SPI_Transmit(0x20);
//    SPI_Transmit(0x0F);
//    while(SPI_Transmit(0x20) != 0x0F);
//
//    // Cấu hình thanh ghi CTRL_REG4
//    SPI_Transmit(0x23);
//    SPI_Transmit(0x80);
//    while(SPI_Transmit(0x23) != 0x80);
//}



