/**
 ******************************************************************************
 * @author: ThanNT
 * @date:   05/09/2016
 * @Update:
 * @AnhHH: Add io function for sth11 sensor.
 ******************************************************************************
**/
#ifndef __IO_CFG_H__
#define __IO_CFG_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"
#include "core_cm4.h"
#include "core_cmFunc.h"

#include "../driver/rtc/rtc.h"
#include "../driver/eeprom/eeprom.h"

#include <stdbool.h>

#include "tm_stm32f4_gpio.h"
#include "tm_stm32f4_pwm.h"

#define SS_DIS GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define SS_EN  GPIO_SetBits(GPIOA, GPIO_Pin_4)
extern void SPI_Configuration(void);
/*
 * define pin for arduino pinMode/digitalWrite/digitalRead
 * NOTE: define value MUST be deferrent
 */

#define SHT1X_CLK_PIN					(0x01)
#define SHT1X_DATA_PIN					(0x02)

#define SSD1306_CLK_PIN					(0x03)
#define SSD1306_DATA_PIN				(0x04)

#define DS1302_CLK_PIN					(0x05)
#define DS1302_DATA_PIN					(0x06)
#define DS1302_CE_PIN					(0x07)

/******************************************************************************
 *Pin map button
*******************************************************************************/
#define BUTTON_MODE_IO_PIN				(GPIO_Pin_3)
#define BUTTON_MODE_IO_PORT				(GPIOB)
#define BUTTON_MODE_IO_CLOCK			(RCC_AHB1Periph_GPIOB)

#define BUTTON_UP_IO_PIN				(GPIO_Pin_4)
#define BUTTON_UP_IO_PORT				(GPIOB)
#define BUTTON_UP_IO_CLOCK				(RCC_AHB1Periph_GPIOB)

#define BUTTON_DOWN_IO_PIN				(GPIO_Pin_13)
#define BUTTON_DOWN_IO_PORT				(GPIOC)
#define BUTTON_DOWN_IO_CLOCK			(RCC_AHB1Periph_GPIOC)

/*****************************************************************************
 *Pin map led life
******************************************************************************/
#define LED_LIFE_IO_PIN					(GPIO_Pin_7)
#define LED_LIFE_IO_PORT				(GPIOA)
#define LED_LIFE_IO_CLOCK				(RCC_AHB1Periph_GPIOA)

/*****************************************************************************
 *Pin map nRF24l01
******************************************************************************/
#define NRF_CE_IO_PIN					(GPIO_Pin_8)
#define NRF_CE_IO_PORT					(GPIOA)
#define NRF_CE_IO_CLOCK					(RCC_AHB1Periph_GPIOA)

#define NRF_CSN_IO_PIN					(GPIO_Pin_9)
#define NRF_CSN_IO_PORT					(GPIOB)
#define NRF_CSN_IO_CLOCK				(RCC_AHB1Periph_GPIOB)

#define NRF_IRQ_IO_PIN					(GPIO_Pin_1)
#define NRF_IRQ_IO_PORT					(GPIOB)
#define NRF_IRQ_IO_CLOCK				(RCC_AHB1Periph_GPIOB)

/*****************************************************************************
 *Pin map Flash W2508
******************************************************************************/
#define FLASH_CE_IO_PIN					(GPIO_Pin_0)
#define FLASH_CE_IO_PORT				(GPIOB)
#define FLASH_CE_IO_CLOCK				(RCC_AHB1Periph_GPIOB)

/*****************************************************************************
 *Pin map IR
******************************************************************************/
#define IR_RX_IO_PIN					(GPIO_Pin_15)
#define IR_RX_IO_PORT					(GPIOA)
#define IR_RX_IO_CLOCK					(RCC_AHB1Periph_GPIOA)

#define IR_TX_IO_PIN					(GPIO_Pin_10)
#define IR_TX_IO_PORT					(GPIOB)
#define IR_TX_IO_CLOCK					(RCC_AHB1Periph_GPIOB)

#define IR_DIR1_IO_PIN					(GPIO_Pin_15)
#define IR_DIR1_IO_PORT					(GPIOB)
#define IR_DIR1_IO_CLOCK				(RCC_AHB1Periph_GPIOB)

#define IR_DIR2_IO_PIN					(GPIO_Pin_11)
#define IR_DIR2_IO_PORT					(GPIOA)
#define IR_DIR2_IO_CLOCK				(RCC_AHB1Periph_GPIOA)

/*****************************************************************************
 *Pin map DS1302
******************************************************************************/
#define DS1302_CE_IO_PIN				(GPIO_Pin_7)
#define DS1302_CE_IO_PORT				(GPIOB)
#define DS1302_CE_IO_CLOCK				(RCC_AHB1Periph_GPIOB)

#define DS1302_DATA_IO_PIN				(GPIO_Pin_6)
#define DS1302_DATA_IO_PORT				(GPIOB)
#define DS1302_DATA_IO_CLOCK			(RCC_AHB1Periph_GPIOB)

#define DS1302_CLK_IO_PIN				(GPIO_Pin_8)
#define DS1302_CLK_IO_PORT				(GPIOB)
#define DS1302_CLK_IO_CLOCK				(RCC_AHB1Periph_GPIOB)

/****************************************************************************
 *Pin map HS1101
*****************************************************************************/
#define HS1101_OUT_PIN					(GPIO_Pin_0)
#define HS1101_OUT_PORT					(GPIOB)
#define HS1101_OUT_CLOCK				(RCC_AHB1Periph_GPIOB)

#define HS1101_IN_PIN					(GPIO_Pin_5)
#define HS1101_IN_PORT					(GPIOB)
#define HS1101_IN_CLOCK					(RCC_AHB1Periph_GPIOB)

/****************************************************************************
 *Pin map CT sensor
*****************************************************************************/
#define CT1_ADC_PIN						(ADC_Channel_0)
#define CT2_ADC_PIN						(ADC_Channel_2)
#define CT3_ADC_PIN						(ADC_Channel_3)
#define CT4_ADC_PIN						(ADC_Channel_4)

#define CT_ADC_PORT						(GPIOA)
#define CT_ADC_CLOCK					(RCC_APB2Periph_ADC1)
#define CT_ADC_IO_CLOCK					(RCC_AHB1Periph_GPIOA)

/****************************************************************************
 *Pin map CT sensor
*****************************************************************************/
#define THER_ADC_PIN					(GPIO_Pin_1)
#define THER_ADC_PORT					(GPIOA)
#define THER_ADC_IO_CLOCK				(RCC_AHB1Periph_GPIOA)

/****************************************************************************
 *Pin map ssd1306
*****************************************************************************/
#define SSD1306_CLK_IO_PIN				(GPIO_Pin_13)
#define SSD1306_CLK_IO_PORT				(GPIOB)
#define SSD1306_CLK_IO_CLOCK			(RCC_AHB1Periph_GPIOB)

#define SSD1306_DATA_IO_PIN				(GPIO_Pin_12)
#define SSD1306_DATA_IO_PORT			(GPIOB)
#define SSD1306_DATA_IO_CLOCK			(RCC_AHB1Periph_GPIOB)

#define SPIx                           SPI2
#define SPIx_CLK                       RCC_APB1Periph_SPI2
#define SPIx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define SPIx_IRQn                      SPI2_IRQn
#define SPIx_IRQHANDLER                SPI2_IRQHandler

#define SPIx_SCK_PIN                   GPIO_Pin_13
#define SPIx_SCK_GPIO_PORT             GPIOB
#define SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define SPIx_SCK_SOURCE                GPIO_PinSource13
#define SPIx_SCK_AF                    GPIO_AF_SPI2

#define SPIx_MISO_PIN                  GPIO_Pin_14
#define SPIx_MISO_GPIO_PORT            GPIOB
#define SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define SPIx_MISO_SOURCE               GPIO_PinSource14
#define SPIx_MISO_AF                   GPIO_AF_SPI2

#define SPIx_MOSI_PIN                  GPIO_Pin_15
#define SPIx_MOSI_GPIO_PORT            GPIOB
#define SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define SPIx_MOSI_SOURCE               GPIO_PinSource15
#define SPIx_MOSI_AF                   GPIO_AF_SPI2
/*****************************************************************************
 *Pin map Encoder PID
*******************************************************************************/
#define	DC1_DIR_PIN							GPIO_Pin_0
#define	DC2_DIR_PIN							GPIO_Pin_1
#define	DC3_DIR_PIN							GPIO_Pin_5
#define	DC4_DIR_PIN							GPIO_Pin_6
#define	DC_DIR_GPIO_PORT					GPIOD
extern TM_PWM_TIM_t TIM08_Data_CH1;
extern TM_PWM_TIM_t TIM08_Data_CH2;
extern TM_PWM_TIM_t TIM08_Data_CH3;
extern TM_PWM_TIM_t TIM08_Data_CH4;
/*****************************************************************************
 *Pin mpu6050
**************************b****************************************************/

//#define TRUE 1
//#define FALSE 0
#define TWI_FREQ 32000L  //changed from 100000L
#define MPU6050_I2C  I2C1
#define dt 2
#define g 9.81
#define Gry_offset 0
#define Gyr_Gain 16.348
#define Angle_offset -0.75
#define pi 3.14159
#define rad2degree 57.3              // Radian to degree conversion
#define Filter_gain 0.912             // e.g.  angle = angle_gyro*Filter_gain + angle_accel*(1-Filter_gain)

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     (MPU6050_ADDRESS_AD0_LOW<<1)

#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75

#define MPU6050_TC_PWR_MODE_BIT     7
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6
#define MPU6050_TC_OTP_BNK_VLD_BIT  0

#define MPU6050_VDDIO_LEVEL_VLOGIC  0
#define MPU6050_VDDIO_LEVEL_VDD     1

#define MPU6050_CFG_EXT_SYNC_SET_BIT    5
#define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6050_CFG_DLPF_CFG_BIT    2
#define MPU6050_CFG_DLPF_CFG_LENGTH 3

#define MPU6050_EXT_SYNC_DISABLED       0x0
#define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACONFIG_XA_ST_BIT           7
#define MPU6050_ACONFIG_YA_ST_BIT           6
#define MPU6050_ACONFIG_ZA_ST_BIT           5
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

#define MPU6050_TEMP_FIFO_EN_BIT    7
#define MPU6050_XG_FIFO_EN_BIT      6
#define MPU6050_YG_FIFO_EN_BIT      5
#define MPU6050_ZG_FIFO_EN_BIT      4
#define MPU6050_ACCEL_FIFO_EN_BIT   3
#define MPU6050_SLV2_FIFO_EN_BIT    2
#define MPU6050_SLV1_FIFO_EN_BIT    1
#define MPU6050_SLV0_FIFO_EN_BIT    0

#define MPU6050_MULT_MST_EN_BIT     7
#define MPU6050_WAIT_FOR_ES_BIT     6
#define MPU6050_SLV_3_FIFO_EN_BIT   5
#define MPU6050_I2C_MST_P_NSR_BIT   4
#define MPU6050_I2C_MST_CLK_BIT     3
#define MPU6050_I2C_MST_CLK_LENGTH  4

#define MPU6050_CLOCK_DIV_348       0x0
#define MPU6050_CLOCK_DIV_333       0x1
#define MPU6050_CLOCK_DIV_320       0x2
#define MPU6050_CLOCK_DIV_308       0x3
#define MPU6050_CLOCK_DIV_296       0x4
#define MPU6050_CLOCK_DIV_286       0x5
#define MPU6050_CLOCK_DIV_276       0x6
#define MPU6050_CLOCK_DIV_267       0x7
#define MPU6050_CLOCK_DIV_258       0x8
#define MPU6050_CLOCK_DIV_500       0x9
#define MPU6050_CLOCK_DIV_471       0xA
#define MPU6050_CLOCK_DIV_444       0xB
#define MPU6050_CLOCK_DIV_421       0xC
#define MPU6050_CLOCK_DIV_400       0xD
#define MPU6050_CLOCK_DIV_381       0xE
#define MPU6050_CLOCK_DIV_364       0xF

#define MPU6050_I2C_SLV_RW_BIT      7
#define MPU6050_I2C_SLV_ADDR_BIT    6
#define MPU6050_I2C_SLV_ADDR_LENGTH 7
#define MPU6050_I2C_SLV_EN_BIT      7
#define MPU6050_I2C_SLV_BYTE_SW_BIT 6
#define MPU6050_I2C_SLV_REG_DIS_BIT 5
#define MPU6050_I2C_SLV_GRP_BIT     4
#define MPU6050_I2C_SLV_LEN_BIT     3
#define MPU6050_I2C_SLV_LEN_LENGTH  4

#define MPU6050_I2C_SLV4_RW_BIT         7
#define MPU6050_I2C_SLV4_ADDR_BIT       6
#define MPU6050_I2C_SLV4_ADDR_LENGTH    7
#define MPU6050_I2C_SLV4_EN_BIT         7
#define MPU6050_I2C_SLV4_INT_EN_BIT     6
#define MPU6050_I2C_SLV4_REG_DIS_BIT    5
#define MPU6050_I2C_SLV4_MST_DLY_BIT    4
#define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6050_MST_PASS_THROUGH_BIT    7
#define MPU6050_MST_I2C_SLV4_DONE_BIT   6
#define MPU6050_MST_I2C_LOST_ARB_BIT    5
#define MPU6050_MST_I2C_SLV4_NACK_BIT   4
#define MPU6050_MST_I2C_SLV3_NACK_BIT   3
#define MPU6050_MST_I2C_SLV2_NACK_BIT   2
#define MPU6050_MST_I2C_SLV1_NACK_BIT   1
#define MPU6050_MST_I2C_SLV0_NACK_BIT   0

#define MPU6050_INTCFG_INT_LEVEL_BIT        7
#define MPU6050_INTCFG_INT_OPEN_BIT         6
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6050_INTCFG_CLKOUT_EN_BIT        0

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_INTLATCH_50USPULSE  0x00
#define MPU6050_INTLATCH_WAITCLEAR  0x01

#define MPU6050_INTCLEAR_STATUSREAD 0x00
#define MPU6050_INTCLEAR_ANYREAD    0x01

#define MPU6050_INTERRUPT_FF_BIT            7
#define MPU6050_INTERRUPT_MOT_BIT           6
#define MPU6050_INTERRUPT_ZMOT_BIT          5
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6050_INTERRUPT_DMP_INT_BIT       1
#define MPU6050_INTERRUPT_DATA_RDY_BIT      0

// TODO: Need to work on DMP related things
#define MPU6050_DMPINT_5_BIT            5
#define MPU6050_DMPINT_4_BIT            4
#define MPU6050_DMPINT_3_BIT            3
#define MPU6050_DMPINT_2_BIT            2
#define MPU6050_DMPINT_1_BIT            1
#define MPU6050_DMPINT_0_BIT            0

#define MPU6050_MOTION_MOT_XNEG_BIT     7
#define MPU6050_MOTION_MOT_XPOS_BIT     6
#define MPU6050_MOTION_MOT_YNEG_BIT     5
#define MPU6050_MOTION_MOT_YPOS_BIT     4
#define MPU6050_MOTION_MOT_ZNEG_BIT     3
#define MPU6050_MOTION_MOT_ZPOS_BIT     2
#define MPU6050_MOTION_MOT_ZRMOT_BIT    0

#define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6050_PATHRESET_GYRO_RESET_BIT    2
#define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6050_PATHRESET_TEMP_RESET_BIT    0

#define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6050_DETECT_FF_COUNT_BIT             3
#define MPU6050_DETECT_FF_COUNT_LENGTH          2
#define MPU6050_DETECT_MOT_COUNT_BIT            1
#define MPU6050_DETECT_MOT_COUNT_LENGTH         2

#define MPU6050_DETECT_DECREMENT_RESET  0x0
#define MPU6050_DETECT_DECREMENT_1      0x1
#define MPU6050_DETECT_DECREMENT_2      0x2
#define MPU6050_DETECT_DECREMENT_4      0x3

#define MPU6050_USERCTRL_DMP_EN_BIT             7
#define MPU6050_USERCTRL_FIFO_EN_BIT            6
#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6050_USERCTRL_DMP_RESET_BIT          3
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2
#define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6050_PWR1_DEVICE_RESET_BIT   7
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_PWR1_CYCLE_BIT          5
#define MPU6050_PWR1_TEMP_DIS_BIT       3
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

#define MPU6050_WAKE_FREQ_1P25      0x0
#define MPU6050_WAKE_FREQ_2P5       0x1
#define MPU6050_WAKE_FREQ_5         0x2
#define MPU6050_WAKE_FREQ_10        0x3

#define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6050_BANKSEL_MEM_SEL_BIT         4
#define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define MPU6050_DMP_MEMORY_BANKS        8
#define MPU6050_DMP_MEMORY_BANK_SIZE    256
#define MPU6050_DMP_MEMORY_CHUNK_SIZE   16

void MPU6050_Initialize();
bool MPU6050_TestConnection();

// GYRO_CONFIG register
uint8_t MPU6050_GetFullScaleGyroRange();
void MPU6050_SetFullScaleGyroRange(uint8_t range);
// ACCEL_CONFIG register
uint8_t MPU6050_GetFullScaleAccelRange();
void MPU6050_SetFullScaleAccelRange(uint8_t range);

// PWR_MGMT_1 register
bool MPU6050_GetSleepModeStatus();
void MPU6050_SetSleepModeStatus(FunctionalState NewState);
void MPU6050_SetClockSource(uint8_t source);
// WHO_AM_I register
uint8_t MPU6050_GetDeviceID();

void MPU6050_GetRawAccelTempGyro(int16_t* AccelGyro);
void MPU6050_OffsetCal();
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);

void MPU6050_I2C_Init();
void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr);
void MPU6050_I2C_BufferRead(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead);

/******************************************************************************
* button function
*******************************************************************************/
extern void io_button_mode_init();
extern void io_button_up_init();
extern void io_button_down_init();

extern uint8_t io_button_mode_read();
extern uint8_t io_button_up_read();
extern uint8_t io_button_down_read();

/******************************************************************************
* led status function
*******************************************************************************/
extern void led_life_init();
extern void led_life_on();
extern void led_life_off();

/******************************************************************************
* flash IO function
*******************************************************************************/
extern void flash_io_ctrl_init();
extern void flash_cs_low();
extern void flash_cs_high();
extern uint8_t flash_transfer(uint8_t);
extern uint8_t SPI1_ReadWriteByte(u8 TxData);
/******************************************************************************
* nfr24l01 IO function
*******************************************************************************/
extern void nrf24l01_io_ctrl_init();
extern void nrf24l01_spi_ctrl_init();
extern void nrf24l01_ce_low();
extern void nrf24l01_ce_high();
extern void nrf24l01_csn_low();
extern void nrf24l01_csn_high();
extern uint8_t nrf24l01_spi_rw(uint8_t);

/******************************************************************************
* ir IO function
*******************************************************************************/
extern void timer_50us_init();
extern void timer_50us_enable();
extern void timer_50us_disable();

extern void ir_rev_io_init();
extern void ir_rev_io_irq_disable();
extern void ir_rev_io_irq_enable();

extern void ir_carrier_freq_init();
extern void ir_carrier_freq_on();
extern void ir_carrier_freq_off();
extern void ir_dir_io_config();
extern void ir_select_direction(uint8_t ir_number);	/* 0 -> 3 */

/******************************************************************************
* shtx IO function
*******************************************************************************/
extern void sht1x_clk_input_mode();
extern void sht1x_clk_output_mode();
extern void sht1x_clk_digital_write_low();
extern void sht1x_clk_digital_write_high();

extern void sht1x_data_input_mode();
extern void sht1x_data_output_mode();
extern void sht1x_data_digital_write_low();
extern void sht1x_data_digital_write_high();
extern int  sht1x_data_digital_read();

/******************************************************************************
* ds1302 IO function
*******************************************************************************/
extern void ds1302_ce_input_mode();
extern void ds1302_ce_output_mode();
extern void ds1302_ce_digital_write_low();
extern void ds1302_ce_digital_write_high();

extern void ds1302_clk_input_mode();
extern void ds1302_clk_output_mode();
extern void ds1302_clk_digital_write_low();
extern void ds1302_clk_digital_write_high();

extern void ds1302_data_input_mode();
extern void ds1302_data_output_mode();
extern void ds1302_data_digital_write_low();
extern void ds1302_data_digital_write_high();

extern uint8_t  ds1302_data_digital_read();

/******************************************************************************
* config DAC, COMP, PWM for read hs1101 function
*******************************************************************************/
extern void io_cfg_dac_hs1101();
extern void io_cfg_comp_hs1101();
extern void io_cfg_timer3_hs1101();
extern void io_cfg_timer4_hs1101();
extern void io_hs1101_read_enable();
extern void io_hs1101_read_disable();
extern void io_reset_timer4_capture();
extern void io_start_timer4_capture();
extern uint32_t io_timer4_get_capture();
extern void GPIO_Configuration();

/******************************************************************************
* config Encoder - PID function
*******************************************************************************/
extern void Move_Up_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent);
extern void Move_Down_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent);
extern void Turn_Release(int16_t DCx, TM_PWM_TIM_t* TIM_Data, TM_PWM_Channel_t Channel, float percent);
extern void ALL_PWM_INIT(void);
extern void ALL_GPIO_INIT(void);
extern void timer4_config(void);
extern void Encoder1_Tim3_Init(void);
extern void Encoder2_Tim2_Init(void);
extern void Encoder3_Tim5_Init(void);
extern void ALL_ENCODER_INIT(void);
extern void ENCODER_Read_Release(float* Save_vitri_feedback, uint32_t* Dir, uint32_t* Cnt, TIM_TypeDef* TIMx);
extern void GPIO_PinMode(GPIO_TypeDef *GPIOx, uint32_t CLOCK, uint16_t GPIO_Pin, GPIOMode_TypeDef GPIO_Mode, GPIOOType_TypeDef GPIO_OType, GPIOSpeed_TypeDef GPIO_Speed, GPIOPuPd_TypeDef GPIO_PuPd);
/******************************************************************************
* adc function
* + themistor sensor
*
* Note: MUST be enable internal clock for adc module.
*******************************************************************************/
/* configure adc peripheral */
extern void io_cfg_adc1(void);

/* adc configure for CT sensor */
extern void adc_ct_io_cfg();
extern uint16_t adc_ct_io_read(uint8_t);

/* adc configure for thermistor sensor */
extern void adc_thermistor_io_cfg();
extern uint16_t adc_thermistor_io_read(uint8_t);

/******************************************************************************
* ssd1306 oled IO function
*******************************************************************************/
extern void ssd1306_clk_input_mode();
extern void ssd1306_clk_output_mode();
extern void ssd1306_clk_digital_write_low();
extern void ssd1306_clk_digital_write_high();

extern void ssd1306_data_input_mode();
extern void ssd1306_data_output_mode();
extern void ssd1306_data_digital_write_low();
extern void ssd1306_data_digital_write_high();
extern int  ssd1306_data_digital_read();

/******************************************************************************
* eeprom function
*******************************************************************************/
#define EEPROM_BASE_ADDRESS         (0x08080000)
#define EEPROM_MAX_SIZE             (0x1000)

extern uint8_t  io_eeprom_read(uint32_t, uint8_t*, uint32_t);
extern uint8_t  io_eeprom_write(uint32_t, uint8_t*, uint32_t);
extern uint8_t  io_eeprom_erase(uint32_t, uint32_t);

/******************************************************************************
* internal flash function
*******************************************************************************/
extern void internal_flash_unlock();
extern void internal_flash_lock();
extern void internal_flash_erase_pages_cal(uint32_t address, uint32_t len);
extern uint8_t internal_flash_write_cal(uint32_t address, uint8_t* data, uint32_t len);
extern void uart(int bau);
extern void windboneFlash_spi_ctrl_init();
extern int16_t MPU6050data[7];
extern int16_t temp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z; // Raw values varaibles
extern float accel_x_OC, accel_y_OC, accel_z_OC, gyro_x_OC,gyro_y_OC, gyro_z_OC; // offset variables
extern float temp_scalled,accel_x_scalled,accel_y_scalled,accel_z_scalled,gyro_x_scalled,gyro_y_scalled,gyro_z_scalled; //Scalled Data varaibles
extern float accel_scale_fact, gyro_scale_fact; // Scale factor variables


extern void io_uart_interface_cfg();
extern void io_uart_interface_transfer(uint8_t);
extern uint8_t io_uart_interface_receiver();

extern void led_test_init();
extern void led_test_on();
extern void led_test_off();
#ifdef __cplusplus
}
#endif

#endif //__IO_CFG_H__
