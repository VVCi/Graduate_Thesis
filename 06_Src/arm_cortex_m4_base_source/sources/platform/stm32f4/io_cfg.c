/**
 ******************************************************************************
 * @author: ThanNT
 * @date:   05/09/2016
 * @Update:
 * @AnhHH: Add io function for sth11 sensor.
 ******************************************************************************
**/
#include <stdint.h>
#include <stdbool.h>

#include "system.h"
#include "io_cfg.h"
#include "stm32f4.h"

#include "arduino/Arduino.h"
#include "../sys/sys_dbg.h"
#include "../app/app_dbg.h"
#include "../common/utils.h"

/******************************************************************************
* button function
*******************************************************************************/
void io_button_mode_init() {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(BUTTON_MODE_IO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = BUTTON_MODE_IO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(BUTTON_MODE_IO_PORT, &GPIO_InitStructure);
}

void io_button_up_init() {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(BUTTON_UP_IO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = BUTTON_UP_IO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(BUTTON_UP_IO_PORT, &GPIO_InitStructure);
}

void io_button_down_init() {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(BUTTON_DOWN_IO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = BUTTON_DOWN_IO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(BUTTON_DOWN_IO_PORT, &GPIO_InitStructure);
}

uint8_t io_button_mode_read() {
	return  GPIO_ReadInputDataBit(BUTTON_MODE_IO_PORT,BUTTON_MODE_IO_PIN);
}

uint8_t io_button_up_read() {
	return  GPIO_ReadInputDataBit(BUTTON_UP_IO_PORT,BUTTON_UP_IO_PIN);
}

uint8_t io_button_down_read() {
	return  GPIO_ReadInputDataBit(BUTTON_DOWN_IO_PORT,BUTTON_DOWN_IO_PIN);
}

/******************************************************************************
* led status function
*******************************************************************************/
void led_life_init() {
	GPIO_InitTypeDef        GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(LED_LIFE_IO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED_LIFE_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_LIFE_IO_PORT, &GPIO_InitStructure);
}

void led_life_on() {
	GPIO_ResetBits(LED_LIFE_IO_PORT, LED_LIFE_IO_PIN);
}

void led_life_off() {
	GPIO_SetBits(LED_LIFE_IO_PORT, LED_LIFE_IO_PIN);
}

void led_test_init() {
	GPIO_InitTypeDef        GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


}
void led_test_on() {
	GPIO_ResetBits(GPIOA, GPIO_Pin_7);
}

void led_test_off() {
	GPIO_SetBits(GPIOA, GPIO_Pin_7);
}
/******************************************************************************
* nfr24l01 IO function
*******************************************************************************/
void nrf24l01_io_ctrl_init() {
	/* CE / CSN / IRQ */
	GPIO_InitTypeDef        GPIO_InitStructure;
#if defined(RF24_USED_POLLING)
#else
	EXTI_InitTypeDef        EXTI_InitStruct;
	NVIC_InitTypeDef        NVIC_InitStruct;
#endif
	/* GPIOA Periph clock enable */
	RCC_AHB1PeriphClockCmd(NRF_CE_IO_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(NRF_CSN_IO_CLOCK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/*CE -> PA8*/
	GPIO_InitStructure.GPIO_Pin = NRF_CE_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(NRF_CE_IO_PORT, &GPIO_InitStructure);

	/*CNS -> PB9*/
	GPIO_InitStructure.GPIO_Pin = NRF_CSN_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(NRF_CSN_IO_PORT, &GPIO_InitStructure);

	/* IRQ -> PB1 */
#if defined(RF24_USED_POLLING)
#else
	GPIO_InitStructure.GPIO_Pin = NRF_IRQ_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(NRF_IRQ_IO_PORT, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);

	EXTI_InitStruct.EXTI_Line = EXTI_Line1;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
#endif
}

void windboneFlash_spi_ctrl_init() {
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/* 100MHz */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI1);

	/*_____________ Mot Kieu De Dinh Trang Thai Chac Chan _______________*/
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);

	/*____________________________________________________________________*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  /* vua truyen vua nha song song*/
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		/* kieu 8 bit >< khi giao tiep giua hai MCU STM nen sd 32b */
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	/* co toi 4 pha ?! */
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		/* chan SS bang Soft ware tuc duoc phep chi dinh >< Hard ware khi chi co 1 Slave => tuc luon truyen cho 1 Slave */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		/* toc do truyen; so bi chia la 48 MHz*/
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	/* bit cao truyen truoc hay thap truyen truoc ! */
	SPI_InitStructure.SPI_CRCPolynomial = 7;	/* chuan Check sum */
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE); /* bit cho phep chay */

	SPI1_ReadWriteByte(0xff); /* dumy data file khong co y nghia */
}

void nrf24l01_spi_ctrl_init() {
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;

	/*!< SPI GPIO Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/*!< SPI Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/*!< Configure SPI pins: SCK */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*!< Configure SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*!< Configure SPI pins: MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect PXx to SPI_SCK */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);

	/* Connect PXx to SPI_MISO */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);

	/* Connect PXx to SPI_MOSI */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);

	/*!< SPI Config */
	SPI_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE); /*!< SPI enable */
}

uint8_t SPI1_ReadWriteByte(u8 TxData)	/* noi dung gui di */
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}

	SPI_I2S_SendData(SPI1, TxData);

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}

	return SPI_I2S_ReceiveData(SPI1);
}

void nrf24l01_ce_low() {
	GPIO_ResetBits(NRF_CE_IO_PORT, NRF_CE_IO_PIN);
}

void nrf24l01_ce_high() {
	GPIO_SetBits(NRF_CE_IO_PORT, NRF_CE_IO_PIN);
}

void nrf24l01_csn_low() {
	GPIO_ResetBits(NRF_CSN_IO_PORT, NRF_CSN_IO_PIN);
}

void nrf24l01_csn_high() {
	GPIO_SetBits(NRF_CSN_IO_PORT, NRF_CSN_IO_PIN);
}

uint8_t nrf24l01_spi_rw(uint8_t data) {
	unsigned long rxtxData = data;
	uint32_t counter;

	/* waiting send idle then send data */
	counter = system_info.cpu_clock / 1000;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
		if (counter-- == 0) {
			FATAL("spi", 0x01);
		}
	}

	SPI_I2S_SendData(SPI1, (uint8_t)rxtxData);

	/* waiting conplete rev data */
	counter = system_info.cpu_clock / 1000;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
		if (counter-- == 0) {
			FATAL("spi", 0x02);
		}
	}

	rxtxData = (uint8_t)SPI_I2S_ReceiveData(SPI1);

	return (uint8_t)rxtxData;
}

/******************************************************************************
* flash IO config
*******************************************************************************/
void flash_io_ctrl_init() {
	GPIO_InitTypeDef        GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(FLASH_CE_IO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = FLASH_CE_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(FLASH_CE_IO_PORT, &GPIO_InitStructure);
	flash_cs_high();
}

void flash_cs_low() {
	GPIO_ResetBits(FLASH_CE_IO_PORT, FLASH_CE_IO_PIN);
}

void flash_cs_high() {
	GPIO_SetBits(FLASH_CE_IO_PORT, FLASH_CE_IO_PIN);
}

uint8_t flash_transfer(uint8_t data) {
	unsigned long rxtxData = data;
	uint32_t counter;

	/* waiting send idle then send data */
	counter = system_info.cpu_clock / 1000;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
		if (counter-- == 0) {
			FATAL("spi", 0x01);
		}
	}

	SPI_I2S_SendData(SPI1, (uint8_t)rxtxData);

	/* waiting conplete rev data */
	counter = system_info.cpu_clock / 1000;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
		if (counter-- == 0) {
			FATAL("spi", 0x02);
		}
	}

	rxtxData = (uint8_t)SPI_I2S_ReceiveData(SPI1);

	return (uint8_t)rxtxData;
}

/* testing spi */
void spi_slave_init() {
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Peripheral Clock Enable -------------------------------------------------*/
	/* Enable the SPI clock */
	SPIx_CLK_INIT(SPIx_CLK, ENABLE);

	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(SPIx_SCK_GPIO_CLK | SPIx_MISO_GPIO_CLK | SPIx_MOSI_GPIO_CLK, ENABLE);

	/* SPI GPIO Configuration --------------------------------------------------*/
	/* GPIO Deinitialisation */
	GPIO_DeInit(SPIx_SCK_GPIO_PORT);
	GPIO_DeInit(SPIx_MISO_GPIO_PORT);
	GPIO_DeInit(SPIx_MOSI_GPIO_PORT);

	/* Connect SPI pins to AF5 */
	GPIO_PinAFConfig(SPIx_SCK_GPIO_PORT, SPIx_SCK_SOURCE, SPIx_SCK_AF);
	GPIO_PinAFConfig(SPIx_MISO_GPIO_PORT, SPIx_MISO_SOURCE, SPIx_MISO_AF);
	GPIO_PinAFConfig(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_SOURCE, SPIx_MOSI_AF);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = SPIx_SCK_PIN;
	GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  SPIx_MISO_PIN;
	GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  SPIx_MOSI_PIN;
	GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStructure);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPIx);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	/* Configure the Priority Group to 1 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Configure the SPI interrupt priority */
	NVIC_InitStructure.NVIC_IRQChannel = SPIx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	/* Slave board configuration */
	/* Initializes the SPI communication */
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_Init(SPIx, &SPI_InitStructure);

	/* Enable the Rx buffer not empty interrupt */
	SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, ENABLE);

	/* Enable the Tx empty interrupt */
	SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, ENABLE);

	/* Enable the SPI peripheral */
	SPI_Cmd(SPIx, ENABLE);

	/* Disable the Rx buffer not empty interrupt */
	SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_RXNE, DISABLE);

	/* Disable the Tx empty interrupt */
	SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_TXE, DISABLE);

	/* Disable the SPI peripheral */
	SPI_Cmd(SPIx, DISABLE);
}

void SPI_Configuration(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	/* SPI_MASTER configuration ------------------------------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	// SPI_SLAVE configuration ------------------------------------------------

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_SPI2);   // only connect to
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);   // only connect to
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);   // only connect to
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);   // only connect to

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);

	SPI_Cmd(SPI2, ENABLE);

	/* Enable SPI_MASTER */
	SPI_Cmd(SPI1, ENABLE);
}




/******************************************************************************
* ir IO function
*******************************************************************************/
void timer_50us_init() {
	TIM_TimeBaseInitTypeDef  timer_50us;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* timer 50us to polling receive IR signal */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	timer_50us.TIM_Period = 1592;
	timer_50us.TIM_Prescaler = 0;
	timer_50us.TIM_ClockDivision = 0;
	timer_50us.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM4, &timer_50us);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM4, DISABLE);
}

void timer_50us_enable() {
	ENTRY_CRITICAL();
	TIM_Cmd(TIM6, ENABLE);
	EXIT_CRITICAL();
}

void timer_50us_disable() {
	ENTRY_CRITICAL();
	TIM_Cmd(TIM6, DISABLE);
	EXIT_CRITICAL();
}

void ir_rev_io_init() {
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* ir read -> PA15*/
	RCC_AHB1PeriphClockCmd(IR_RX_IO_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = IR_RX_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(IR_RX_IO_PORT, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);

	EXTI_InitStruct.EXTI_Line = EXTI_Line15;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void ir_rev_io_irq_disable() {
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line15;
	EXTI_InitStruct.EXTI_LineCmd = DISABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStruct);
}

void ir_rev_io_irq_enable() {
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line15;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStruct);
}

void ir_carrier_freq_init() {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHB1PeriphClockCmd( IR_TX_IO_CLOCK, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 841;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 281;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM2, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);

	TIM_Cmd(TIM2, DISABLE);
}

void ir_carrier_freq_on() {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* enable PWM output GPIO */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin   = IR_TX_IO_PIN;
	GPIO_Init(IR_TX_IO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(IR_TX_IO_PORT, GPIO_PinSource10, GPIO_AF_TIM2);

	/* enable timer */
	ENTRY_CRITICAL();
	TIM_Cmd(TIM2, ENABLE);
	EXIT_CRITICAL();
}

void ir_carrier_freq_off() {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* PWM output GPIO LOW level */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin   = IR_TX_IO_PIN;
	GPIO_Init(IR_TX_IO_PORT, &GPIO_InitStructure);

	GPIO_ResetBits(IR_TX_IO_PORT, IR_TX_IO_PIN);

	/* disable timer */
	ENTRY_CRITICAL();
	TIM_Cmd(TIM2, DISABLE);
	EXIT_CRITICAL();
}

/******************************************************************************
* direction IR IO function
*******************************************************************************/
void ir_dir_io_config(){
	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(IR_DIR1_IO_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = IR_DIR1_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(IR_DIR1_IO_PORT, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(IR_DIR2_IO_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = IR_DIR2_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(IR_DIR2_IO_PORT, &GPIO_InitStructure);
}

void ir_select_direction(uint8_t ir_number){
	switch (ir_number) {
	case 0:
		GPIO_SetBits(IR_DIR1_IO_PORT,IR_DIR1_IO_PIN);
		GPIO_SetBits(IR_DIR2_IO_PORT,IR_DIR2_IO_PIN);
		break;

	case 1:
		GPIO_ResetBits(IR_DIR1_IO_PORT,IR_DIR1_IO_PIN);
		GPIO_SetBits(IR_DIR2_IO_PORT,IR_DIR2_IO_PIN);
		break;

	case 2:
		GPIO_ResetBits(IR_DIR1_IO_PORT,IR_DIR1_IO_PIN);
		GPIO_ResetBits(IR_DIR2_IO_PORT,IR_DIR2_IO_PIN);
		break;

	case 3:
		GPIO_SetBits(IR_DIR1_IO_PORT,IR_DIR1_IO_PIN);
		GPIO_ResetBits(IR_DIR2_IO_PORT,IR_DIR2_IO_PIN);
		break;

	default:
		break;
	}
}

/******************************************************************************
* sht11 IO function
*******************************************************************************/
void sht1x_clk_input_mode(){
	GPIO_InitTypeDef    GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void sht1x_clk_output_mode(){
	GPIO_InitTypeDef    GPIO_InitStructure;

	/* GPIOA Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void sht1x_clk_digital_write_low() {
	GPIO_ResetBits(GPIOA, GPIO_Pin_11);
}

void sht1x_clk_digital_write_high(){
	GPIO_SetBits(GPIOA, GPIO_Pin_11);
}

void sht1x_data_input_mode(){
	GPIO_InitTypeDef    GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void sht1x_data_output_mode(){
	GPIO_InitTypeDef    GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void sht1x_data_digital_write_low(){
	GPIO_ResetBits(GPIOA, GPIO_Pin_12);
}

void sht1x_data_digital_write_high(){
	GPIO_SetBits(GPIOA, GPIO_Pin_12);
}

int sht1x_data_digital_read(){
	return (int)(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12));
}

/******************************************************************************
* ds1302 IO function
*******************************************************************************/
/* rst pin config*/
void ds1302_ce_input_mode() {
	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(DS1302_CE_IO_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = DS1302_CE_IO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(DS1302_CE_IO_PORT, &GPIO_InitStructure);
}

void ds1302_ce_output_mode() {
	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(DS1302_CE_IO_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = DS1302_CE_IO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(DS1302_CE_IO_PORT, &GPIO_InitStructure);
}

void ds1302_ce_digital_write_low() {
	GPIO_ResetBits(DS1302_CE_IO_PORT, DS1302_CE_IO_PIN);
}

void ds1302_ce_digital_write_high(){
	GPIO_SetBits(DS1302_CE_IO_PORT, DS1302_CE_IO_PIN);
}

/* scl pin config*/
void ds1302_clk_input_mode() {
	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(DS1302_CLK_IO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = DS1302_CLK_IO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(DS1302_CLK_IO_PORT, &GPIO_InitStructure);
}

void ds1302_clk_output_mode() {
	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(DS1302_CLK_IO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = DS1302_CLK_IO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(DS1302_CLK_IO_PORT, &GPIO_InitStructure);
}

void ds1302_clk_digital_write_low() {
	GPIO_ResetBits(DS1302_CLK_IO_PORT, DS1302_CLK_IO_PIN);
}

void ds1302_clk_digital_write_high(){
	GPIO_SetBits(DS1302_CLK_IO_PORT, DS1302_CLK_IO_PIN);
}

/* sda pin config*/
void ds1302_data_input_mode() {
	GPIO_InitTypeDef    GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_AHB1PeriphClockCmd(DS1302_DATA_IO_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = DS1302_DATA_IO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(DS1302_DATA_IO_PORT, &GPIO_InitStructure);
}

void ds1302_data_output_mode() {
	GPIO_InitTypeDef    GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(DS1302_DATA_IO_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = DS1302_DATA_IO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(DS1302_DATA_IO_PORT, &GPIO_InitStructure);
}

void ds1302_data_digital_write_low() {
	GPIO_ResetBits(DS1302_DATA_IO_PORT, DS1302_DATA_IO_PIN);
}

void ds1302_data_digital_write_high() {
	GPIO_SetBits(DS1302_DATA_IO_PORT, DS1302_DATA_IO_PIN);
}

uint8_t ds1302_data_digital_read(){
	return GPIO_ReadInputDataBit(DS1302_DATA_IO_PORT, DS1302_DATA_IO_PIN);
}

/******************************************************************************
* hs1101 IO function
* config DAC, COMP, PWM for read hs1101 function
*******************************************************************************/
void io_cfg_dac_hs1101() {
	DAC_InitTypeDef DAC_InitStructure;

	RCC_LSICmd(ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	/* DAC channel1 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	/* Enable DAC Channel1 */
	DAC_Cmd(DAC_Channel_1, DISABLE);

	/* Set DAC Channel1 DHR register
	 * VREF ~ 3.3 V
	 * DAC_OUT = (3.3 * 2588) / 4095 ~> 2.86 V = 3.3*(1-exp(-t/R*C)) with t=R*C
	 * Set DAC Value = 2588
	*/
	DAC_SetChannel1Data(DAC_Align_12b_R, 2588);
}

void io_cfg_comp_hs1101() {
	//	COMP_InitTypeDef COMP_InitStructure;
	//	GPIO_InitTypeDef GPIO_InitStructure;

	//	RCC_AHB1PeriphClockCmd( HS1101_IN_CLOCK, ENABLE);
	//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_COMP, ENABLE);
	//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	//	GPIO_InitStructure.GPIO_Pin   = HS1101_IN_PIN;
	//	GPIO_Init(HS1101_IN_PORT, &GPIO_InitStructure);

	//	COMP_InitStructure.COMP_InvertingInput = COMP_InvertingInput_DAC1;
	//	COMP_InitStructure.COMP_OutputSelect   = COMP_OutputSelect_TIM4IC4;
	//	COMP_InitStructure.COMP_Speed		   = COMP_Speed_Fast;
	//	COMP_Init(&COMP_InitStructure);

	//	/* Close the I/O analog switch number n */
	//	SYSCFG_RIIOSwitchConfig(RI_IOSwitch_GR6_2, ENABLE);
	//	COMP_WindowCmd(DISABLE);
	//	COMP_Cmd(DISABLE);
}

void io_cfg_timer3_hs1101() {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	uint16_t period = (uint16_t)((SystemCoreClock) / 1100);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd( HS1101_OUT_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin   = HS1101_OUT_PIN;
	GPIO_Init(HS1101_OUT_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(HS1101_OUT_PORT, GPIO_PinSource0, GPIO_AF_TIM3);

	TIM_TimeBaseStructure.TIM_Period        = period;
	TIM_TimeBaseStructure.TIM_Prescaler     = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_Pulse       = period/2;
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);

	TIM4->SR = 0x00;

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_SelectOutputTrigger(TIM3,TIM_TRGOSource_Enable);
	TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3,DISABLE);
}

void io_cfg_timer4_hs1101() {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	uint16_t period = (uint16_t)((SystemCoreClock) / 1100);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_Channel		= TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter    = 0;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	TIM4->SR = 0x00;

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Trigger);
	TIM_SelectInputTrigger(TIM4, TIM_TS_ITR3);

	TIM_ITConfig(TIM4,TIM_IT_CC4,ENABLE);
	TIM_Cmd(TIM4, DISABLE);
}

/******************************************************************************
* adc function
* + CT sensor
* + themistor sensor
* Note: MUST be enable internal clock for adc module.
*******************************************************************************/
void io_cfg_adc1(void) {
	ADC_InitTypeDef ADC_InitStructure;
	RCC_HSICmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	/* Enable ADC1 clock */
	RCC_APB2PeriphClockCmd(CT_ADC_CLOCK , ENABLE);
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode =DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 2;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);
}

void adc_ct_io_cfg() {
	GPIO_InitTypeDef    GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(CT_ADC_IO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = CT1_ADC_PIN | CT2_ADC_PIN | CT3_ADC_PIN | CT4_ADC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CT_ADC_PORT, &GPIO_InitStructure);
}

void adc_thermistor_io_cfg() {
	GPIO_InitTypeDef    GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(THER_ADC_IO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = THER_ADC_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(THER_ADC_PORT, &GPIO_InitStructure);
}

uint16_t adc_ct_io_read(uint8_t chanel) {
	ADC_RegularChannelConfig(ADC1, chanel, 1, ADC_SampleTime_3Cycles);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}

uint16_t adc_thermistor_io_read(uint8_t chanel) {
	ADC_RegularChannelConfig(ADC1, chanel, 1, ADC_SampleTime_3Cycles);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}

/******************************************************************************
* ssd1306 oled IO function
*******************************************************************************/
void ssd1306_clk_input_mode() {
	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(SSD1306_CLK_IO_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = SSD1306_CLK_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SSD1306_CLK_IO_PORT, &GPIO_InitStructure);
}

void ssd1306_clk_output_mode() {
	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(SSD1306_CLK_IO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = SSD1306_CLK_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SSD1306_CLK_IO_PORT, &GPIO_InitStructure);
}

void ssd1306_clk_digital_write_low() {
	GPIO_ResetBits(SSD1306_CLK_IO_PORT,SSD1306_CLK_IO_PIN);
}

void ssd1306_clk_digital_write_high() {
	GPIO_SetBits(SSD1306_CLK_IO_PORT,SSD1306_CLK_IO_PIN);
}

void ssd1306_data_input_mode() {
	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(SSD1306_DATA_IO_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = SSD1306_DATA_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SSD1306_DATA_IO_PORT, &GPIO_InitStructure);
}

void ssd1306_data_output_mode() {
	GPIO_InitTypeDef    GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(SSD1306_DATA_IO_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = SSD1306_DATA_IO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SSD1306_DATA_IO_PORT, &GPIO_InitStructure);
}

void ssd1306_data_digital_write_low() {
	GPIO_ResetBits(SSD1306_DATA_IO_PORT, SSD1306_DATA_IO_PIN);
}

void ssd1306_data_digital_write_high() {
	GPIO_SetBits(SSD1306_DATA_IO_PORT, SSD1306_DATA_IO_PIN);
}

int ssd1306_data_digital_read() {
	return (int)(GPIO_ReadInputDataBit(SSD1306_DATA_IO_PORT, SSD1306_DATA_IO_PIN));
}

void internal_flash_unlock() {
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR);
}

void internal_flash_lock() {
	FLASH_Lock();
}

void internal_flash_erase_pages_cal(uint32_t address, uint32_t len) {
	uint32_t page_number;
	uint32_t index;

	page_number = len / 256;

	if ((page_number * 256) < len) {
		page_number++;
	}

	for (index = 0; index < page_number; index++) {
		//FLASH_ErasePage(address + (index * 256));
	}
}

uint8_t internal_flash_write_cal(uint32_t address, uint8_t* data, uint32_t len) {
	uint32_t temp;
	uint32_t index = 0;
	FLASH_Status flash_status = FLASH_BUSY;

	while (index < len) {
		temp = 0;

		memcpy(&temp, &data[index], (len - index) >= sizeof(uint32_t) ? sizeof(uint32_t) : (len - index));

		//flash_status = FLASH_FastProgramWord(address + index, temp);

		if(flash_status == FLASH_COMPLETE) {
			index += sizeof(uint32_t);
		}
		else {
			FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR);
		}
	}

	return 1;
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure PB0 PB1 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void uart(int bau) {

	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStructure; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
		 * note that only USART1 and USART6 are connected to APB2
		 * the other USARTs are connected to APB1
		 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by
		 * USART1, PB6 for TX and PB7 for RX
		 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins
		 * so they work correctly with the USART1 peripheral
		 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
		 * so that the USART1 can take over control of the
		 * pins
		 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* Now the USART_InitStruct is used to define the
		 * properties of USART1
		 */
	USART_InitStructure.USART_BaudRate = bau;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);	// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
		 * and the interrupt controller is configured
		 * to jump to the USART1_IRQHandler() function
		 * if the USART1 receive interrupt occurs
		 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);


}

/** @defgroup MPU6050_Library
* @{
*/

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050_Initialize()
{
	MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
	MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_1000);
	MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_16);
	MPU6050_SetSleepModeStatus(DISABLE);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool MPU6050_TestConnection()
{
	if(MPU6050_GetDeviceID() == 0x34) //0b110100; 8-bit representation in hex = 0x34
		return 1;
	else
		return 0;
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t MPU6050_GetDeviceID()
{
	uint8_t tmp;
	MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
	return tmp;
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050_SetClockSource(uint8_t source)
{
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(uint8_t range)
{
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleGyroRange()
{
	uint8_t tmp;
	MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
	return tmp;
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleAccelRange()
{
	uint8_t tmp;
	MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
	return tmp;
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 */
void MPU6050_SetFullScaleAccelRange(uint8_t range)
{
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050_GetSleepModeStatus()
{
	uint8_t tmp;
	MPU6050_ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
	if(tmp == 0x00)
		return 0;
	else
		return 1;
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_SetSleepModeStatus(FunctionalState NewState)
{
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

/** Get raw 6-axis motion sensor readings (accel/temp/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 7
 * @see MPU6050_RA_ACCEL_XOUT_H
 * data 0 -> 2 : Accel
 * data 3      : Temp
 * data 4 -> 6 : Gyro
 */

void MPU6050_GetRawAccelTempGyro(int16_t* AccelGyro)
{
	uint8_t dataR[14],i;
	MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, dataR, MPU6050_RA_ACCEL_XOUT_H, 14);

	for(i=0;i<7;i++) {AccelGyro[i]=(dataR[i*2]<<8)|dataR[i*2+1];}

	// AccelGyro[i]=(dataR[i*2]<<8)|dataR[i*2+1];
	accel_x = (dataR[0*2]<<8)|dataR[0*2+1];
	accel_y = (dataR[1*2]<<8)|dataR[1*2+1];
	accel_z = (dataR[2*2]<<8)|dataR[2*2+1];

	temp = (dataR[3*2]<<8)|dataR[3*2+1];

	gyro_x = (dataR[4*2]<<8)|dataR[4*2+1];
	gyro_y = (dataR[5*2]<<8)|dataR[5*2+1];
	gyro_z = (dataR[6*2]<<8)|dataR[6*2+1];
}
/*
void MPU6050_GetRawAccelTempGyro(s16* AccelGyro)
{
	u8 dataR[14],i;
	MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, dataR, MPU6050_RA_ACCEL_XOUT_H, 14);

	for(i=0;i<7;i++)
	AccelGyro[i]=(dataR[i*2]<<8)|dataR[i*2+1];

}
*/





/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t tmp, mask;
	MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
	mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tmp &= ~(mask); // zero all important bits in existing byte
	tmp |= data; // combine data with existing byte
	MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);
}
/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
	uint8_t tmp;
	MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
	tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
	MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);
}
/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	uint8_t tmp, mask;
	MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
	mask = ((1 << length) - 1) << (bitStart - length + 1);
	tmp &= mask;
	tmp >>= (bitStart - length + 1);
	*data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
	uint8_t tmp;
	MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
	*data = tmp & (1 << bitNum);
}

/**
* @brief  Initializes the I2C peripheral used to drive the MPU6050
* @param  None
* @return None
*/
void MPU6050_I2C_Init()
{

	GPIO_InitTypeDef  GPIO_InitStructure;
	I2C_InitTypeDef   I2C_InitStructure;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Reset I2C peripheral */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	/* Release reset signal of I2C IP */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

	/*!< GPIO configuration */
	/* I2C_SCL*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	/* I2C_SDA*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	/*!< Configure I2C pins: SCL */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 40000;

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
	I2C_Init(I2C1, &I2C_InitStructure);

}

/**
* @brief  Writes one byte to the  MPU6050.
* @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
* @param  writeAddr : address of the register in which the data will be written
* @return None
*/
void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr)
{
	//  ENTR_CRT_SECTION();

	/* Send START condition */
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear	 it */
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Send the MPU6050's internal address to write to */
	I2C_SendData(MPU6050_I2C, writeAddr);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send the byte to be written */
	I2C_SendData(MPU6050_I2C, *pBuffer);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP condition */
	I2C_GenerateSTOP(MPU6050_I2C, ENABLE);

	// EXT_CRT_SECTION();

}

/**
* @brief  Reads a block of data from the MPU6050.
* @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
* @param  readAddr : MPU6050's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
* @return None
*/

void MPU6050_I2C_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead)
{
	// ENTR_CRT_SECTION();

	/* While the bus is busy */
	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY));

	/* Send START condition */
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(MPU6050_I2C, ENABLE);

	/* Send the MPU6050's internal address to write to */
	I2C_SendData(MPU6050_I2C, readAddr);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STRAT condition a second time */
	I2C_GenerateSTART(MPU6050_I2C, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send MPU6050 address for read */
	I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	/* While there is data to be read */
	while(NumByteToRead)
	{
		if(NumByteToRead == 1)
		{
			/* Disable Acknowledgement */
			I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);

			/* Send STOP Condition */
			I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
		}

		/* Test on EV7 and clear it */
		if(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			/* Read a byte from the MPU6050 */
			*pBuffer = I2C_ReceiveData(MPU6050_I2C);

			/* Point to the next location where the byte read will be saved */
			pBuffer++;

			/* Decrement the read bytes counter */
			NumByteToRead--;
		}
	}

	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);
}

void MPU6050_OffsetCal(){
	int x=0,y=0,z=0,i;

	MPU6050_GetRawAccelTempGyro(MPU6050data);
	MPU6050_GetRawAccelTempGyro(MPU6050data);

	// Gyro Offset Calculation
	x=gyro_x;
	y=gyro_y;
	z=gyro_z;

	for (i=1;i<=500;i++){
		MPU6050_GetRawAccelTempGyro(MPU6050data);
		x=(x+gyro_x)/2;
		y=(y+gyro_y)/2;
		z=(z+gyro_z)/2;
	}
	gyro_x_OC=x;
	gyro_y_OC=y;
	gyro_z_OC=z;
	// Accel Offset Calculation

	x=accel_x;
	y=accel_y;
	z=accel_z;

	for (i=1;i<=500;i++){
		MPU6050_GetRawAccelTempGyro(MPU6050data);
		x=(x+accel_x)/2;
		y=(y+accel_y)/2;
		z=(z+accel_z)/2;
	}

	accel_x_OC = x;
	accel_y_OC = y;
	accel_z_OC = z-(float)g*1000/accel_scale_fact;
}

void io_uart_interface_cfg() {
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStructure; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);	// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART2 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART2_IRQHandler() function
	 * if the USART2 receive interrupt occurs
	 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	// finally this enables the complete USART2 peripheral
	USART_Cmd(USART2, ENABLE);
}

void io_uart_interface_transfer(uint8_t c) {
	/* wait tx data buf empty */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);

	USART_SendData(USART2, (uint8_t)c);

	/* wait tx data transfer completed */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

uint8_t io_uart_interface_receiver() {
	uint8_t c = 0;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		c = (uint8_t)USART_ReceiveData(USART2);

	}

	return c;
}
/******************************************************************************
* Encoder PID IO function
*******************************************************************************/
/*----------------------------------------------------------------------------*/
/*
*********************************************************************************
*  							Encoder_1  TIM3 PA6 PA7								*
*																				*
*********************************************************************************
*/
void Encoder1_Tim3_Init(void) {
	TIM_TimeBaseInitTypeDef  					TIM_TimeBaseStructure;
	TIM_ICInitTypeDef        					TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	GPIO_PinMode(GPIOA, RCC_AHB1Periph_GPIOA,
				 GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_AF,
				 GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM3, 0);
	TIM_Cmd(TIM3, ENABLE);
}

/********************************************************************************
*  							Encoder_2  TIM2 PA15 PB3                            *
*																				*
*********************************************************************************
*/
void Encoder2_Tim2_Init(void) {
	TIM_TimeBaseInitTypeDef  					TIM_TimeBaseStructure;
	TIM_ICInitTypeDef        					TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	GPIO_PinMode(GPIOA, RCC_AHB1Periph_GPIOA,
				 GPIO_Pin_15, GPIO_Mode_AF,
				 GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP);
	GPIO_PinMode(GPIOB, RCC_AHB1Periph_GPIOB,
				 GPIO_Pin_3, GPIO_Mode_AF,
				 GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
	//TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);


	//TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
	TIM_SetCounter(TIM2, 1000000);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM2, ENABLE);
}

/********************************************************************************
*  								Encoder_3  TIM5 PA0 PA1                         *
*																				*
*********************************************************************************
*/
void Encoder3_Tim5_Init(void) {
	TIM_TimeBaseInitTypeDef  					TIM_TimeBaseStructure;
	TIM_ICInitTypeDef        					TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	GPIO_PinMode(GPIOA, RCC_AHB1Periph_GPIOA,
				 GPIO_Pin_0 | GPIO_Pin_1, GPIO_Mode_AF,
				 GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0000;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 10;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM5, 2000000000);
	TIM_Cmd(TIM5, ENABLE);
}

/*
*********************************************************************************
*																				*
*  						         INIT ALL ENCODER								*
*																				*
*********************************************************************************
*/
void ALL_ENCODER_INIT(void) {
	/* DC1 --> Encoder  TIM3  PA6  PA7   */
	//Encoder1_Tim3_Init();
	/* DC2 --> Encoder  TIM2  PA15 PB3   */
	//Encoder2_Tim2_Init();
	/* DC3 --> Encoder  TIM5  PA0  PA1   */
	Encoder3_Tim5_Init();
}
/********************************************************************************
*																				*
*				 Dir: huong cua dong co -> 1 :dem len, 0 : dem xuong			*
*																				*
*       *Cnt = TIMx->CNT;  <==> TIM_GetCounter(TIMx);  ->  dem encoder			*
*																				*
*				*Save_vitri_feedback -> con tro toi vi tri doc duoc				*
*																				*
*********************************************************************************
*/
void ENCODER_Read_Release(float* Save_vitri_feedback, uint32_t* Dir, uint32_t* Cnt, TIM_TypeDef* TIMx) {

	*Dir = ((TIMx->CR1 & TIM_CR1_DIR)? (0) : 1);         // huong cua dong co //1 la dem len, 0 la dem xuong

	*Cnt = TIMx->CNT;     //TIM_GetCounter(TIMx);       // dem encoder

	*Save_vitri_feedback = ((*Cnt - 2000000000.0) * 360) / 4000000.0;               // export to degree

}

/*
*********************************************************************************
*																				*
*  						         INIT TEMP GPIO									*
*																				*
*********************************************************************************
*/
void GPIO_PinMode(GPIO_TypeDef *GPIOx, uint32_t CLOCK, uint16_t GPIO_Pin, GPIOMode_TypeDef GPIO_Mode, GPIOOType_TypeDef GPIO_OType, GPIOSpeed_TypeDef GPIO_Speed, GPIOPuPd_TypeDef GPIO_PuPd) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
	GPIO_InitStructure.GPIO_OType = GPIO_OType;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOx, GPIO_Pin);
	return;
}

/******************************************************************************
* Init timer for read speed and position for Encoder
*******************************************************************************/
void timer4_config(void)    // cau hinh cho ngat 0.02 s
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	NVIC_InitTypeDef			NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	uint16_t PrescalerValue = 0;

	/*	TIM_Prescaler = ((SystemCoreClock/n) / F_timer) - 1
	*	T = (TIM_Period * TIM_Prescaler) / (SystemCoreClock/n)
		20ms = 0.02s = (10000 * TIM_Prescaler) / 84*10^6
	=> TIM_Prescaler = 168 => F_timer = 500kHz					*/

	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 500000) - 1;

	TIM_DeInit(TIM4);
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 9999;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue; // 168 - 1
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//TIM_ARRPreloadConfig(TIM4,TIM_UpdateSource_Regular);

	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

	TIM_ClearFlag(TIM4, TIM_FLAG_Update);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	TIM_Cmd (TIM4, ENABLE);


	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
}
/*
*************************************************************************************
	*                           INIT ALL GPIO                                       *
	*
	*DC1_DIR_PIN							PD0										*
	*DC2_DIR_PIN							PD1										*
	*DC3_DIR_PIN							PD5										*
	*DC4_DIR_PIN							PD6										*
	*
*************************************************************************************
*/
void ALL_GPIO_INIT(void) {
	TM_GPIO_Init(DC_DIR_GPIO_PORT, DC1_DIR_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(DC_DIR_GPIO_PORT, DC2_DIR_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	//TM_GPIO_Init(DC_DIR_GPIO_PORT, DC3_DIR_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	//TM_GPIO_Init(DC_DIR_GPIO_PORT, DC4_DIR_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
}

/*
*************************************************************************************
	*                           INIT ALL PWM                                        *
	*																				*
	*PWM_1						TIM 8 - CH1						PC6					*
	*PWM_2						TIM 8 - CH2						PC7					*
	*PWM_3						TIM 8 - CH3						PC8					*
	*PWM_4						TIM 8 - CH4						PC9					*
	*																				*
*************************************************************************************
*/

TM_PWM_TIM_t TIM08_Data_CH1;
TM_PWM_TIM_t TIM08_Data_CH2;
TM_PWM_TIM_t TIM08_Data_CH3;
TM_PWM_TIM_t TIM08_Data_CH4;

void ALL_PWM_INIT(void)
{
	/* Set PWM to 1KHz frequency on timer TIM 8 - CH1 */
	/* 1KHz = ?ms = ?us */
	//TM_PWM_InitTimer(TIM8, &TIM08_Data_CH1, 1000);

	/* Initialize PWM on TIM8, Channel 1 and PinsPack 1 = PC6*/
	//TM_PWM_InitChannel(&TIM08_Data_CH1, TM_PWM_Channel_1, TM_PWM_PinsPack_1);
	/* Set default duty cycle */
	//TM_PWM_SetChannelPercent(&TIM08_Data_CH1, TM_PWM_Channel_1, 100);

	/* Set PWM to 1KHz frequency on timer TIM 8 - CH2 */
	/* 1KHz = ?ms = ?us */
	TM_PWM_InitTimer(TIM8, &TIM08_Data_CH2, 1000);

	/* Initialize PWM on TIM8, Channel 2 and PinsPack 1 =  PC7*/
	TM_PWM_InitChannel(&TIM08_Data_CH2, TM_PWM_Channel_2, TM_PWM_PinsPack_1);
	/* Set default duty cycle */
	TM_PWM_SetChannelPercent(&TIM08_Data_CH2, TM_PWM_Channel_2, 100);

	/* Set PWM to 1KHz frequency on timer TIM 8 - CH3 */
	/* 1KHz = ?ms = ?us */
	//TM_PWM_InitTimer(TIM8, &TIM08_Data_CH3, 1000);

	/* Initialize PWM on TIM8, Channel 3 and PinsPack 1 =  PC8*/
	//TM_PWM_InitChannel(&TIM08_Data_CH3, TM_PWM_Channel_3, TM_PWM_PinsPack_1);
	/* Set default duty cycle */
	//TM_PWM_SetChannelPercent(&TIM08_Data_CH3, TM_PWM_Channel_3, 100);

	/* Set PWM to 1KHz frequency on timer TIM 8 - CH4 */
	/* 1KHz = ?ms = ?us */
	//TM_PWM_InitTimer(TIM8, &TIM08_Data_CH4, 1000);

	/* Initialize PWM on TIM8, Channel 4 and PinsPack 1 =  PC9*/
	//TM_PWM_InitChannel(&TIM08_Data_CH4, TM_PWM_Channel_4, TM_PWM_PinsPack_1);
	/* Set default duty cycle */
	//TM_PWM_SetChannelPercent(&TIM08_Data_CH4, TM_PWM_Channel_4, 100);
}


/*
*****************************************************************************************
*							FUNCTION CHO DONG CO QUAY THUAN																*
*																																												*
*****************************************************************************************
*/
void Move_Up_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent)
{
	switch(DCx)
	{
	case 1:
		TM_GPIO_SetPinHigh(DC_DIR_GPIO_PORT, DC1_DIR_PIN);
		TM_PWM_SetChannelPercent(TIM_Data, Channel, 100 - percent);
		break;

	case 2:
		/*TM_GPIO_SetPinHigh(DC_DIR_GPIO_PORT, DC1_DIR_PIN);
		TM_GPIO_SetPinLow(DC_DIR_GPIO_PORT, DC2_DIR_PIN);*/
		TM_GPIO_SetPinLow(DC_DIR_GPIO_PORT, DC1_DIR_PIN);
		TM_GPIO_SetPinHigh(DC_DIR_GPIO_PORT, DC2_DIR_PIN);

		TM_PWM_SetChannelPercent(TIM_Data, Channel, 100 - percent);
		break;

	case 3:
		TM_GPIO_SetPinHigh(DC_DIR_GPIO_PORT, DC3_DIR_PIN);
		TM_PWM_SetChannelPercent(TIM_Data, Channel, 100 - percent);
		break;

	case 4:
		TM_GPIO_SetPinHigh(DC_DIR_GPIO_PORT, DC4_DIR_PIN);
		TM_PWM_SetChannelPercent(TIM_Data, Channel, 100 - percent);
		break;

	default:
		break;
	}

}

void Move_Down_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent) {

	switch(DCx)
	{
	case 1:
		TM_GPIO_SetPinLow(DC_DIR_GPIO_PORT, DC1_DIR_PIN);
		TM_PWM_SetChannelPercent(TIM_Data, Channel, 100 - percent);
		break;

	case 2:
		TM_GPIO_SetPinHigh(DC_DIR_GPIO_PORT, DC1_DIR_PIN);
		TM_GPIO_SetPinLow(DC_DIR_GPIO_PORT, DC2_DIR_PIN);
		TM_PWM_SetChannelPercent(TIM_Data, Channel, 100 - percent);
		break;

	case 3:
		TM_GPIO_SetPinLow(DC_DIR_GPIO_PORT, DC3_DIR_PIN);
		TM_PWM_SetChannelPercent(TIM_Data, Channel, 100 - percent);
		break;

	case 4:
		TM_GPIO_SetPinLow(DC_DIR_GPIO_PORT, DC4_DIR_PIN);
		TM_PWM_SetChannelPercent(TIM_Data, Channel, 100 - percent);
		break;

	default:
		break;
	}

}

void Turn_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent) {

	if(percent > 0) {
		Move_Up_Release(DCx, TIM_Data, Channel, percent);
	}

	if(percent < 0) {
		Move_Down_Release(DCx, TIM_Data, Channel,(-percent));
	}

}
