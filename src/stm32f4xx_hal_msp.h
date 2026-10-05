#ifndef STM32F4XX_HAL_MSP_H
#define STM32F4XX_HAL_MSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

// Core MSP Initialization
void HAL_MspInit(void);

// Peripheral Hardware Initialization Overrides
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c);
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm);

// Timer Pin Configuration (Required by MX_TIM5_Init)
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_MSP_H */