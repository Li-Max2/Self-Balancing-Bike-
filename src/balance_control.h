#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float accel_angle;
    float bike_angle;
    float gyro_rate;
} BalanceState;

void Balance_Init(float accel_y, float accel_z);
void Balance_Update(float accel_y, float accel_z, float gyro_x, float dt);
const BalanceState *Balance_GetState(void);

#endif
#ifdef __cplusplus
}
#endif