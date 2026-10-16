// ICM20948.h
#ifndef ICM20948_H
#define ICM20948_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"
#include "stm32hal_init.h"

// register locations
#define ICM20948_ADDR (0x69 << 1) // shifted left 1 b/c of sum pin thing (need to check)
#define ICM20948_WHO_AM_I 0x00
#define ICM20948_WHO_AM_I_EXPECTED 0xEA
#define ICM20948_BANK_SEL 0x7F
#define ICM20948_PWR_MGMT_1 0x06
#define ICM20948_PWR_MGMT_2 0x07
#define ICM20948_GYRO_CONFIG_1 0x01
#define ICM20948_GYRO_SMPLRT_DIV 0x00
#define ICM20948_ACCEL_CONFIG 0x14
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_ACCEL_XOUT_H 0x2D
#define ICM20948_GYRO_XOUT_H 0x33

// pin defs
#define LED_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_13 

// struct for accelerator and gyroscope variables
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
} ICM20948_Data;

// function declarations
void ICM20948_Select_Bank(uint8_t bank);
void ICM20948_WriteReg(uint8_t bank, uint8_t reg, uint8_t val);
uint8_t ICM20948_ReadReg(uint8_t bank, uint8_t reg);
void ICM20948_Init(void);
void ICM20948_Calibrate(void);
int ICM20948_Read(ICM20948_Data *data);

#ifdef __cplusplus
}
#endif

#endif