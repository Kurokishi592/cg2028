/**
******************************************************************************
* @file    radio_target.h
* @author  Matthew Mielke
* @version V1.0.0
* @date    07-Jul-2021
* @brief     Defines several macros allowing the user to easily modify the
*          digital interface to the SPIRIT1 transceiver.
******************************************************************************
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RADIO_SPI_H
#define __RADIO_SPI_H

/* Includes ------------------------------------------------------------------*/
// #include "spi.h"
#include "../../../../../../Core/Inc/main.h"
/* Exported constants --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef *radioSpi = &hspi3;

#define RADIO_GPIO_IRQ        SPSGRF_915_GPIO3_EXTI5_EXTI_IRQn
#define SPI_ENTER_CRITICAL()  HAL_NVIC_DisableIRQ(RADIO_GPIO_IRQ);
#define SPI_EXIT_CRITICAL()   HAL_NVIC_EnableIRQ(RADIO_GPIO_IRQ);

#define RadioSpiCSLow()       HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_RESET)
#define RadioSpiCSHigh()      HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET)

#define RADIO_SPI_TIMEOUT_MAX ((uint32_t)1000)

#define RADIO_SDN_PORT        SPSGRF_915_SDN_GPIO_Port
#define RADIO_SDN_PIN         SPSGRF_915_SDN_Pin

#endif /*__RADIO_SPI_H */
