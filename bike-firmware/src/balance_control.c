#include "balance_control.h"
#include <math.h>

#define DEG_TO_RAD 0.01745329252f
#define FILTER_TAU 0.25f
#define GYRO_FILTER_TAU 0.01f // How fast to move towards the new gyro reading

float zero_offset_angle = 0;
static BalanceState state;


void Balance_Init(float accel_y, float accel_z)
{
    zero_offset_angle = atan2f(accel_y, -accel_z); // Converts accel y and z values into an angle offset for calibration
    state.accel_angle = 0.0f;
    state.bike_angle = 0.0f;
    state.gyro_rate = 0.0f;
}

void Balance_Update(float accel_y, float accel_z, float gyro_x, float dt){

    state.accel_angle = atan2f(accel_y, -accel_z) - zero_offset_angle; // converts current accel values into a usable angle

    float raw_gyro = gyro_x * DEG_TO_RAD; // deg -> radian conversion

    // gyro low-pass filter
    float gyro_alpha = dt / (GYRO_FILTER_TAU + dt); 
    state.gyro_rate += gyro_alpha*(raw_gyro - state.gyro_rate);// gradually moving filtered gyro to raw gyro (~1/3 every loop currently)

    float alpha = FILTER_TAU / (FILTER_TAU + dt);
    state.bike_angle = alpha * (state.bike_angle + state.gyro_rate * dt)+ (1.0f - alpha) * state.accel_angle; // bike angle prediction equation
}

const BalanceState *Balance_GetState(void)
{
    return &state;
}