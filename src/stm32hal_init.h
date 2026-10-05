// stm32hal_init.h
#ifndef stm32hal_init_h
#define stm32hal_init_h

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdio.h>

//type defs
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart2;

//private defines (hal peripherals in main.h of stm32 ide)
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SPI2_CSN_Pin GPIO_PIN_12
#define SPI2_CSN_GPIO_Port GPIOB
#define SPI2_CE_Pin GPIO_PIN_4
#define SPI2_CE_GPIO_Port GPIOB

// Private function prototypes (function declarations)
int _write(int fd, char* ptr, int len);
void MX_GPIO_Init();
void MX_USART2_UART_Init();
void MX_SPI2_Init();
void MX_TIM5_Init();
void MX_I2C2_Init();
#ifdef __cplusplus
}
#endif

#endif