/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI_EXT_SCK_Pin GPIO_PIN_2
#define SPI_EXT_SCK_GPIO_Port GPIOE
#define VDD_3V3_SENSORS_EN_Pin GPIO_PIN_3
#define VDD_3V3_SENSORS_EN_GPIO_Port GPIOE
#define SPI_EXT_NS_Pin GPIO_PIN_4
#define SPI_EXT_NS_GPIO_Port GPIOE
#define SPI_EXT_MISO_Pin GPIO_PIN_5
#define SPI_EXT_MISO_GPIO_Port GPIOE
#define SPI_EXT_MOSI_Pin GPIO_PIN_6
#define SPI_EXT_MOSI_GPIO_Port GPIOE
#define FMU_GPIO2_Pin GPIO_PIN_13
#define FMU_GPIO2_GPIO_Port GPIOC
#define FMU_OE1_Pin GPIO_PIN_14
#define FMU_OE1_GPIO_Port GPIOC
#define FMU_LED3_Pin GPIO_PIN_15
#define FMU_LED3_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define MPU_DRDY_Pin GPIO_PIN_0
#define MPU_DRDY_GPIO_Port GPIOC
#define FMU_GPIO4_Pin GPIO_PIN_1
#define FMU_GPIO4_GPIO_Port GPIOC
#define MPU_CS_Pin GPIO_PIN_2
#define MPU_CS_GPIO_Port GPIOC
#define FMU_LED2_Pin GPIO_PIN_3
#define FMU_LED2_GPIO_Port GPIOC
#define SBUS_OUTPUT_Pin GPIO_PIN_0
#define SBUS_OUTPUT_GPIO_Port GPIOA
#define SBUS_INPUT_Pin GPIO_PIN_1
#define SBUS_INPUT_GPIO_Port GPIOA
#define BAT_C_SENS_Pin GPIO_PIN_2
#define BAT_C_SENS_GPIO_Port GPIOA
#define BAT_V_SENS_Pin GPIO_PIN_3
#define BAT_V_SENS_GPIO_Port GPIOA
#define VDD_5V_SENS_Pin GPIO_PIN_4
#define VDD_5V_SENS_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_5
#define SPI_SCK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define FMU_LED1_Pin GPIO_PIN_4
#define FMU_LED1_GPIO_Port GPIOC
#define PRESSURE_SENS_Pin GPIO_PIN_5
#define PRESSURE_SENS_GPIO_Port GPIOC
#define FMU_CH9_Pin GPIO_PIN_0
#define FMU_CH9_GPIO_Port GPIOB
#define FMU_CH10_Pin GPIO_PIN_1
#define FMU_CH10_GPIO_Port GPIOB
#define FMU_BOOT1_DOWN_Pin GPIO_PIN_2
#define FMU_BOOT1_DOWN_GPIO_Port GPIOB
#define UART7_RX_Pin GPIO_PIN_7
#define UART7_RX_GPIO_Port GPIOE
#define UART7_TX_Pin GPIO_PIN_8
#define UART7_TX_GPIO_Port GPIOE
#define FMU_CH1_Pin GPIO_PIN_9
#define FMU_CH1_GPIO_Port GPIOE
#define FMU_OE3_Pin GPIO_PIN_10
#define FMU_OE3_GPIO_Port GPIOE
#define FMU_CH2_Pin GPIO_PIN_11
#define FMU_CH2_GPIO_Port GPIOE
#define FMU_LED4_Pin GPIO_PIN_12
#define FMU_LED4_GPIO_Port GPIOE
#define FMU_CH3_Pin GPIO_PIN_13
#define FMU_CH3_GPIO_Port GPIOE
#define FMU_CH4_Pin GPIO_PIN_14
#define FMU_CH4_GPIO_Port GPIOE
#define SDIO_IPS_Pin GPIO_PIN_15
#define SDIO_IPS_GPIO_Port GPIOE
#define I2C_SCL_Pin GPIO_PIN_10
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_11
#define I2C_SDA_GPIO_Port GPIOB
#define FRAM_SCK_Pin GPIO_PIN_13
#define FRAM_SCK_GPIO_Port GPIOB
#define FRAM_MISO_Pin GPIO_PIN_14
#define FRAM_MISO_GPIO_Port GPIOB
#define FRAM_MOSI_Pin GPIO_PIN_15
#define FRAM_MOSI_GPIO_Port GPIOB
#define USART3_TX_Pin GPIO_PIN_8
#define USART3_TX_GPIO_Port GPIOD
#define USART3_RX_Pin GPIO_PIN_9
#define USART3_RX_GPIO_Port GPIOD
#define FRAM_CS_Pin GPIO_PIN_10
#define FRAM_CS_GPIO_Port GPIOD
#define FMU_GPIO5_Pin GPIO_PIN_11
#define FMU_GPIO5_GPIO_Port GPIOD
#define FMU_GPIO7_Pin GPIO_PIN_12
#define FMU_GPIO7_GPIO_Port GPIOD
#define FMU_CH6_Pin GPIO_PIN_13
#define FMU_CH6_GPIO_Port GPIOD
#define FMU_CH5_Pin GPIO_PIN_14
#define FMU_CH5_GPIO_Port GPIOD
#define MAG_DRDY_Pin GPIO_PIN_15
#define MAG_DRDY_GPIO_Port GPIOD
#define ALARM_Pin GPIO_PIN_6
#define ALARM_GPIO_Port GPIOC
#define FMU_CH8_Pin GPIO_PIN_7
#define FMU_CH8_GPIO_Port GPIOC
#define SDIO_D0_Pin GPIO_PIN_8
#define SDIO_D0_GPIO_Port GPIOC
#define SDIO_D1_Pin GPIO_PIN_9
#define SDIO_D1_GPIO_Port GPIOC
#define FMU_OE2_Pin GPIO_PIN_8
#define FMU_OE2_GPIO_Port GPIOA
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define FMU_GPIO6_Pin GPIO_PIN_10
#define FMU_GPIO6_GPIO_Port GPIOA
#define USB_FS_DM_Pin GPIO_PIN_11
#define USB_FS_DM_GPIO_Port GPIOA
#define USB_FS_DP_Pin GPIO_PIN_12
#define USB_FS_DP_GPIO_Port GPIOA
#define FMU_SWDIO_Pin GPIO_PIN_13
#define FMU_SWDIO_GPIO_Port GPIOA
#define FMU_SWCLK_Pin GPIO_PIN_14
#define FMU_SWCLK_GPIO_Port GPIOA
#define FMU_CH12_Pin GPIO_PIN_15
#define FMU_CH12_GPIO_Port GPIOA
#define SDIO_D2_Pin GPIO_PIN_10
#define SDIO_D2_GPIO_Port GPIOC
#define SDIO_D3_Pin GPIO_PIN_11
#define SDIO_D3_GPIO_Port GPIOC
#define SDIO_CK_Pin GPIO_PIN_12
#define SDIO_CK_GPIO_Port GPIOC
#define FMU_GPIO8_Pin GPIO_PIN_0
#define FMU_GPIO8_GPIO_Port GPIOD
#define FMU_LED7_Pin GPIO_PIN_1
#define FMU_LED7_GPIO_Port GPIOD
#define SDIO_CMD_Pin GPIO_PIN_2
#define SDIO_CMD_GPIO_Port GPIOD
#define FMU_GPIO1_Pin GPIO_PIN_3
#define FMU_GPIO1_GPIO_Port GPIOD
#define FMU_GPIO3_Pin GPIO_PIN_4
#define FMU_GPIO3_GPIO_Port GPIOD
#define USART2_TX_Pin GPIO_PIN_5
#define USART2_TX_GPIO_Port GPIOD
#define USART2_RX_Pin GPIO_PIN_6
#define USART2_RX_GPIO_Port GPIOD
#define BARO_CS_Pin GPIO_PIN_7
#define BARO_CS_GPIO_Port GPIOD
#define FMU_CH11_Pin GPIO_PIN_3
#define FMU_CH11_GPIO_Port GPIOB
#define FMU_CH7_Pin GPIO_PIN_4
#define FMU_CH7_GPIO_Port GPIOB
#define FMU_LED6_Pin GPIO_PIN_5
#define FMU_LED6_GPIO_Port GPIOB
#define FMU_LED5_Pin GPIO_PIN_7
#define FMU_LED5_GPIO_Port GPIOB
#define SBUS_OUT_EN_Pin GPIO_PIN_8
#define SBUS_OUT_EN_GPIO_Port GPIOB
#define PPM_INPUT_Pin GPIO_PIN_9
#define PPM_INPUT_GPIO_Port GPIOB
#define UART8_RX_Pin GPIO_PIN_0
#define UART8_RX_GPIO_Port GPIOE
#define UART8_TX_Pin GPIO_PIN_1
#define UART8_TX_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
