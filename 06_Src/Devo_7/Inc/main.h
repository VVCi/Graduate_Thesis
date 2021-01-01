/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define RELAY_0_Pin GPIO_PIN_0
#define RELAY_0_GPIO_Port GPIOC
#define RELAY_1_Pin GPIO_PIN_1
#define RELAY_1_GPIO_Port GPIOC
#define RELAY_2_Pin GPIO_PIN_2
#define RELAY_2_GPIO_Port GPIOC
#define RELAY_3_Pin GPIO_PIN_3
#define RELAY_3_GPIO_Port GPIOC
#define AUX2_Pin GPIO_PIN_9
#define AUX2_GPIO_Port GPIOE
#define AUX2_EXTI_IRQn EXTI9_5_IRQn
#define ELEV_Pin GPIO_PIN_10
#define ELEV_GPIO_Port GPIOE
#define ELEV_EXTI_IRQn EXTI15_10_IRQn
#define AILE_Pin GPIO_PIN_11
#define AILE_GPIO_Port GPIOE
#define AILE_EXTI_IRQn EXTI15_10_IRQn
#define THRO_Pin GPIO_PIN_12
#define THRO_GPIO_Port GPIOE
#define THRO_EXTI_IRQn EXTI15_10_IRQn
#define RUDD_Pin GPIO_PIN_13
#define RUDD_GPIO_Port GPIOE
#define RUDD_EXTI_IRQn EXTI15_10_IRQn
#define GEAR_Pin GPIO_PIN_14
#define GEAR_GPIO_Port GPIOE
#define GEAR_EXTI_IRQn EXTI15_10_IRQn
#define AUX1_Pin GPIO_PIN_15
#define AUX1_GPIO_Port GPIOE
#define AUX1_EXTI_IRQn EXTI15_10_IRQn
#define PWM_1_Pin GPIO_PIN_6
#define PWM_1_GPIO_Port GPIOC
#define PWM_2_Pin GPIO_PIN_7
#define PWM_2_GPIO_Port GPIOC
#define PWM_3_Pin GPIO_PIN_8
#define PWM_3_GPIO_Port GPIOC
#define PWM_4_Pin GPIO_PIN_9
#define PWM_4_GPIO_Port GPIOC
#define DIR_1_Pin GPIO_PIN_0
#define DIR_1_GPIO_Port GPIOD
#define DIR_2_Pin GPIO_PIN_1
#define DIR_2_GPIO_Port GPIOD
#define DIR_3_Pin GPIO_PIN_5
#define DIR_3_GPIO_Port GPIOD
#define DIR_4_Pin GPIO_PIN_6
#define DIR_4_GPIO_Port GPIOD

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
  #ifdef STM32H743xx
    #define __HAL_GPIO_SET_GPIO(__SET_GPIO__, __GPIO_PIN__) (__SET_GPIO__->BSRRL = __GPIO_PIN__)
    #define __HAL_GPIO_RESET_GPIO(__SET_GPIO__, __GPIO_PIN__) (__SET_GPIO__->BSRRH = __GPIO_PIN__)
  #else
    #define __HAL_GPIO_READ_GPIO(__SET_GPIO__, __GPIO_PIN__) (__SET_GPIO__->IDR & __GPIO_PIN__)
    #define __HAL_GPIO_SET_GPIO(__SET_GPIO__, __GPIO_PIN__) (__SET_GPIO__->BSRR = __GPIO_PIN__)
    #define __HAL_GPIO_RESET_GPIO(__SET_GPIO__, __GPIO_PIN__) (__SET_GPIO__->BSRR = (uint32_t)__GPIO_PIN__<<16U)
    #define __MY_HAL_TIM_DISABLE(__HANDLE__) (__HANDLE__)->Instance->CR1 &= ~(TIM_CR1_CEN)
    #define __MY_HAL_TIM_SET_AUTORELOAD(__HANDLE__, __AUTORELOAD__) ((__HANDLE__)->Instance->ARR = (__AUTORELOAD__))
    #define __HAL_TIM_SET_COMPARE1(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR1 = (__COMPARE__))
    #define __HAL_TIM_SET_COMPARE2(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR2 = (__COMPARE__))
    #define __HAL_TIM_SET_COMPARE3(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR3 = (__COMPARE__))
    #define __HAL_TIM_SET_COMPARE4(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR4 = (__COMPARE__))
  #endif
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
