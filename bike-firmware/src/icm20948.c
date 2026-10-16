// ICM20948.c
#include "ICM20948.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


float g_gx_offset = 0, g_gy_offset = 0, g_gz_offset = 0;
float g_ax_offset = 0, g_ay_offset = 0, g_az_offset = 0;

void ICM20948_Init(void)
{
    char msg[64];
    uint8_t whoami = 0;
    uint8_t attempts = 0;

    // Trying to read the device 10 times to ensure it is working
    // Whoami address is the test read register - should return 0xEA for ICM20948
    while(attempts < 10)
    {
        HAL_I2C_Mem_Read(&hi2c2, ICM20948_ADDR, ICM20948_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &whoami, 1, 50);
        printf("Attempt %d WHO_AM_I: 0x%02X\r\n", attempts, whoami);
        
        if(whoami == ICM20948_WHO_AM_I_EXPECTED) {
            printf("IMU found!\r\n");
            break;
        }
        
        HAL_Delay(50);
        attempts++;
    }
    
    // Once knowing device is working, continue setup
    // Wake up the chip - clear sleep bit
    ICM20948_WriteReg(0, ICM20948_PWR_MGMT_1, 0x01); // auto clock select
    HAL_Delay(100);  // let it wake up


    // REGISTER SETUP STUFF
    // Enable accel and gyro
    ICM20948_WriteReg(0, ICM20948_PWR_MGMT_2, 0x00); // all sensors on

    // Configure gyro - Bank 2
    ICM20948_WriteReg(2, ICM20948_GYRO_CONFIG_1, 0b00111011); // +- 500dps, DLPF on setting 7 - 3DB BW 361.4 HZ (max reading)
    ICM20948_WriteReg(2, ICM20948_GYRO_SMPLRT_DIV, 0x00); // Max rate

    // Configure accel - Bank 2
    ICM20948_WriteReg(2, ICM20948_ACCEL_CONFIG, 0b00011001); // +-2g, DLPF on
    ICM20948_WriteReg(2, ICM20948_ACCEL_SMPLRT_DIV_1, 0x00); // max rate

    // Back to bank 0 for reading data
    ICM20948_Select_Bank(0);

    printf("ICM20948 Init Done\r\n");
}

// Just for bank selection
void ICM20948_Select_Bank(uint8_t bank)
{
    // Left shift by 4 because in REG_BANK_SEL register, bank num selection uses bits [5:4]
	uint8_t val = (bank << 4);
	HAL_I2C_Mem_Write(&hi2c2, ICM20948_ADDR, ICM20948_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

// Helper function for writing to a register
void ICM20948_WriteReg(uint8_t bank, uint8_t reg, uint8_t val)
{
    ICM20948_Select_Bank(bank);
    HAL_I2C_Mem_Write(&hi2c2, ICM20948_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

// Helper function for reading from a register
uint8_t ICM20948_ReadReg(uint8_t bank, uint8_t reg)
{
    uint8_t val = 0;
    ICM20948_Select_Bank(bank);
    HAL_I2C_Mem_Read(&hi2c2, ICM20948_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    return val;
}

// READING DATA FROM IMU
int ICM20948_Read(ICM20948_Data *data)
{
    // read 12 bytes starting from ACCEL_XOUT_H
    // covers accel X,Y,Z and gyro X,Y,Z (2 bytes each, respetive of order)
    //ICM20948_Select_Bank(0);

    // ONE I2c burst read for all 12 bytes, automatically goes to next register after each read, storing in buf
    uint8_t buf[12];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, ICM20948_ADDR, ICM20948_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buf, 12, 5); //all in one fucntion because it pulse reads from one register to the next
    
    if(status != HAL_OK)
    {
        printf("I2C FAIL status:%d error:0x%08lX state:0x%02X\r\n", status, HAL_I2C_GetError(&hi2c2), HAL_I2C_GetState(&hi2c2));
        return 0;
    }

    // Combining high and low bytes into 16-bit signed integers through shifting
    int16_t ax = (int16_t)(buf[0] << 8 | buf[1]);
    int16_t ay = (int16_t)(buf[2] << 8 | buf[3]);
    int16_t az = (int16_t)(buf[4] << 8 | buf[5]);
    int16_t gx = (int16_t)(buf[6] << 8 | buf[7]);
    int16_t gy = (int16_t)(buf[8] << 8 | buf[9]);
    int16_t gz = (int16_t)(buf[10] << 8 | buf[11]);

    // converting to real units 
    // +- 16g range → sensitivity = 16384 LSB/g (divide by 16384 to get 2g range for 16 signed bit read)
    // REMOVED OFFSET FOR ACCEL OF RNOW
    //data->accel_x = ax / 16384.0f - g_ax_offset;
    //data->accel_y = ay / 16384.0f - g_ay_offset;
    //data->accel_z = az / 16384.0f - g_az_offset; // no z offset 
    data->accel_x = ax / 16384.0f;
    data->accel_y = ay / 16384.0f;
    data->accel_z = az / 16384.0f; // no z offset 
    // +- 500dps range -> sensitivity = 65.5 LSB/dps
    data->gyro_x = gx / 65.5f - g_gx_offset;
    data->gyro_y = gy / 65.5f - g_gy_offset;
    data->gyro_z = gz / 65.5f - g_gz_offset;

    return 1;
}

void ICM20948_Calibrate(void)
{
	printf("Calibrating...\r\n");
	// Resetting previous callibration offsets for new callibration
	g_gx_offset = 0; g_gy_offset = 0; g_gz_offset = 0;
	g_ax_offset = 0; g_ay_offset = 0; g_az_offset = 0;
    printf("Offsets reset\r\n");

    // Testing single read time to ensure I2C is working properly
    ICM20948_Data test;
    uint32_t t1 = HAL_GetTick();
    ICM20948_Read(&test);
    uint32_t t2 = HAL_GetTick();
    printf("Single read took: %lu ms\r\n", t2 - t1);


    // Summing variables for cailbration
	float sumGx = 0, sumGy = 0, sumGz = 0;
	float sumAx = 0, sumAy = 0, sumAz = 0;

	int samples = 500; // Number of values to average
	ICM20948_Data raw; // Struct for raw data

	// Summing up all error values
	for(int i = 0; i < samples; i++)
	{
		ICM20948_Read(&raw);
		sumGx += raw.gyro_x;
		sumGy += raw.gyro_y;
		sumGz += raw.gyro_z;
		sumAx += raw.accel_x;
		sumAy += raw.accel_y;
        sumAz += raw.accel_z;

		//led blinking for calibration
		if(i % 50 == 0)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}

		HAL_Delay(4);  // 4 m/s * 500 steps = ~2 second total for calibration

	}

	 // Dividing for average error val per step
	 g_gx_offset = sumGx / samples;
	 g_gy_offset = sumGy / samples;
	 g_gz_offset = sumGz / samples;
	 g_ax_offset = sumAx / samples;
	 g_ay_offset = sumAy / samples;
     g_az_offset = sumAz / samples;

	 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	 printf("Cal done. Gx:%.3f Gy:%.3f Gz:%.3f Ax:%.3f Ay:%.3f Az:%.3f\r\n", g_gx_offset, g_gy_offset, g_gz_offset, g_ax_offset, g_ay_offset, g_az_offset);
	 HAL_Delay(1000);
}
