#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>
#include <SimpleFOCDrivers.h>
#include <SPI.h>
#include "encoders/as5048a/MagneticSensorAS5048A.h"

extern "C" {
#include "stm32f4xx_hal.h"
}

#include "icm20948.h"
#include "stm32hal_init.h"
#include "NRF24L01.h"
#include "balance_control.h"
#define ENC_CS PB10


// CONTROL VARIABLES
#define REM_K1 4.9f
#define REM_K2 0.9f
#define REM_K3 0.000f
#define U_MAX 6.0f

#define REM_K4 0.000f

#define BALANCE_PERIOD_US 5000UL // 5000 us = 0.005s. 1/0.005 = 200 HZ cycle time
#define CONTROL_PERIOD_US 10000UL

// SETUP VARIABLES 
// Arduino pins for motor driver and ecoder (everything else is handled by STM HAL)
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver(PA8, PA9, PA10, PA7);
MagneticSensorAS5048A sensor(ENC_CS);

// Recieve address for nrf24L01
uint8_t RxAddress[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};

// Holds all IMU data (accel xyz, gyro xyz)
ICM20948_Data g_imu;

// Balance stuct holding accel angle, bike angle, and gyro rate calculated from the IMU data and filter
const BalanceState *balance;

// Timing vars
uint32_t last_balance_time = 0;
uint32_t last_angle_print_time = 0;


// Running IMU/filter at 200 Hz

void calibrateBike();
void CheckEmergencyStop();

// SETUP
void setup() {
    // Wait for peripherals to stabilize
    HAL_Delay(500); 
    
    // Stm32 peripheral init
    MX_GPIO_Init();
    MX_TIM5_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_SPI2_Init();
    
    // IMU init and calibration
    ICM20948_Init();
    //ICM20948_Calibrate();
    
    // NRF24L01 init and setup
    NRF24_Init();
    NRF24_RxMode(RxAddress, 10); // setting NRF24 to recieve with address and channel 10


    // SPI STEUP
    SPI.setSCLK(PB3);
    SPI.setMISO(PB4);
    SPI.setMOSI(PB5);
    SPI.begin();

    sensor.init(&SPI);

    motor.linkSensor(&sensor);
    
    // SIMPLE FOC motor driver setup
    driver.voltage_power_supply = 12.0;
    driver.pwm_frequency = 25000;
    driver.init();
    driver.enable();
    motor.linkDriver(&driver);
    
    // Motor tuning
    //motor.controller = MotionControlType::velocity;
    //motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;   
    motor.controller = MotionControlType::torque;


    // PID tuning
    motor.voltage_sensor_align = 7.0f; 
    motor.voltage_limit = 7.0f;       
    //motor.velocity_limit = 25.0f;     
    //motor.PID_velocity.P = 0.04f;     
    //motor.PID_velocity.I = 1.5f;      
    //motor.PID_velocity.D = 0.0f;      
    // motor.LPF_velocity.Tf = 0.02f;    

    // Motor and FOC initialization
    motor.init();
    motor.initFOC();
    motor.enable();

    // Timer for servo PWM
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); // PWM for servo on tim5 CH 1

    //setup for back wheel motor
    analogWriteFrequency(20000);  // set frequency for back motor

    pinMode(PA4, OUTPUT); // Enable/start 
    pinMode(PA5, OUTPUT); // PWM pin
    pinMode(PA6, OUTPUT); // Direction
    
    digitalWrite(PA4, HIGH); // Enable motor
    analogWrite(PA5, 255); // Default not moving
    digitalWrite(PA6, LOW); // FORWARD to start

    printf("Setup done!\r\n");

    motor.disable();

    // Accelerometer calibration for balance point accross 200 samples
    calibrateBike();

    // Getting pointer for balance state struct into main
    balance = Balance_GetState();

    motor.enable();

    printf("Balance zero calibrated\r\n");
    last_balance_time = micros();
}

// ALL VARIABLES FOR MAIN CONTROL LOOP

//TIMINGS ________________
uint32_t last_print_time = 0;
uint32_t last_servo_time = 0;
uint32_t last_nrf_time = 0;
uint32_t last_control_time = 0;
uint32_t now_us = 0;

// WII NUNCHUCK STUFF ______________________
uint32_t nunchuck_val = 0; // placeholder for nunchuk joystick value
uint8_t Nunchuck_Data[32]; //data to recieve into (0-x, 1-y, 2-c + z)
uint8_t nunchuck_x_val = 118; // nunchuk joystick x value
uint8_t nunchuck_y_val = 118; // nunchuk joystick y value
uint8_t nunchuck_c_val = 0; // nunchuk c val
uint8_t nunchuck_z_val = 0; // nunchuk joystick z value

// SERVO STUFF FOR STEERING _____________________
#define SERVO_CENTER 1500 // Center pulse position for servo
#define SERVO_MIN 600 //
#define SERVO_MAX 2400
uint16_t servo_pulse_us = SERVO_CENTER; // middle servo position

// Back wheel speed variables ___________________________________
int16_t drive_speed = 0; // 0 = stopped, - reverse, + forward (current motor speed)
int16_t target_speed = 0; // target speed we want to get to (converted from raw nunchuck speed)
int16_t raw_speed = 255; // raw speed (255 = stop, 0 = full speed)
uint8_t pwm_send = 255; // final PWM send to motor

#define MAX_SPEED 180 // Max speed for back wheel (250 = stop, 0 = full speed)

// Safety limit for how fast to deccelerate/accelerate motor per loop cycle so it's not instant
#define ACTIVE_RAMP 10 // Active accelerate or deccelerate joystick input
#define PASSIVE_RAMP 5 // No joystick input (back to 0)

int8_t ramp_speed = 5; // Safety for how fast to get from drive_speed to target_speed so motor doesnt instantly switch speeds
bool target_forward;


// CONTROL VARIABLES _______________
float dt = 0; // Time between control loops
float u = 0; // Motor torque variable

int imu_fail_count = 0;
static bool vertical = false; // Safety flag for if bike is upright (verical = upright)

// Balance State variables
float angle_deg = 0.0f;
float gyro_deg = 0.0f;
float angle_abs = 0.0f;

float m_speed = 0.0f; // not in use currently 
float rem_wheel_vel = motor.shaft_velocity;

// Control equation parameters ____________________
float angle_term = REM_K1 * angle_deg;
float gyro_term = REM_K2 * gyro_deg;
float wheel_term = REM_K3 * rem_wheel_vel;
float accum_term = REM_K4 * m_speed;
float angle_raw = 0.0f;

bool emergency_stop = false;
uint8_t uart_rx_char = 0;


// MAIN CONTROL LOOP
void loop()
{
    CheckEmergencyStop();
    motor.loopFOC();
    now_us = micros();
    
    // BALANCE CONTROL LOOP
    if ((uint32_t)(now_us - last_balance_time) >= BALANCE_PERIOD_US)
    {

        dt = (now_us - last_balance_time) / 1000000.0f; // Calculating the real time elapsed (us) and converting it to seconds
        last_balance_time = now_us; // Rsetting last balance time

        // SUCCESSFUL IMU READ
        if (ICM20948_Read(&g_imu))
        {
            imu_fail_count = 0; // Reset IMU fail count

            // Updating the balance struct to get angle and bike fall speed
            Balance_Update(g_imu.accel_y, g_imu.accel_z, g_imu.gyro_x, dt);

            // Current angle and gyro vals in deg/s (filter is in radians 1 rad = ~57 deg)
            angle_deg = balance->bike_angle * 57.29578f;
            gyro_deg = balance->gyro_rate * 57.29578f;

            // Set vertical flag to false if bike fell
            if (fabsf(angle_deg) > 20.0f){
                vertical = false;
            }
            // Renable if within 0.5 deg of upright again
            else if (!vertical && (fabsf(angle_deg) < 0.5f)){
                vertical = true;
            }

            if ((uint32_t)(now_us - last_control_time) >= CONTROL_PERIOD_US)
            {
                last_control_time = now_us;

                // If upright
                if (vertical){
                    // CONTROL STUFF 
                    rem_wheel_vel = motor.shaft_velocity;

                    angle_term = REM_K1 * angle_deg;
                    gyro_term = REM_K2 * gyro_deg;
                    wheel_term = REM_K3 * rem_wheel_vel;

                    u = angle_term + gyro_term + wheel_term;
                    u = constrain(u, -U_MAX, U_MAX);
                }
                else{
                    u = 0.0f;
                    angle_raw = 0.0f;
                    angle_term = 0.0f;
                    gyro_term = 0.0f;
                }
            }
        }
        // OTHERWISE IMU FAILED TO READ OVER I2C
        else
        {
            imu_fail_count++;

            if (imu_fail_count >= 3)
            {
                u = 0.0f; // Stop applying torque if no new IMU data is read in a very short period
            }

            if (imu_fail_count >= 20)
            {
                // RESET i2c if imu keeps failing
                HAL_I2C_DeInit(&hi2c1);
                HAL_Delay(2);
                MX_I2C1_Init();

                imu_fail_count = 0;
            }
        }
    }

    // FINALLY send updated motor torque command
    // keep as -u 
    if (!emergency_stop)
    {
        motor.move(-u);
    }
    else
    {
        u = 0.0f;
    }
    //motor.move(0.0f);
    
    //print debugging stuff 
    
    if (HAL_GetTick() - last_angle_print_time >= 200)
    {
        last_angle_print_time = HAL_GetTick();
        printf("Ang:%5.2f Gy:%6.2f Vel:%7.2f U:%5.2f Uq:%5.2f\r\n", angle_deg,gyro_deg, motor.shaft_velocity, u, motor.voltage.q);
    }
    

  // Servo and nrf inputs for steering and back wheel motor_______________________
    
  if (HAL_GetTick() - last_nrf_time >= 10) {
    last_nrf_time = HAL_GetTick();
    
    if (isDataAvailable(1) == 1) {
        NRF24_Receive(Nunchuck_Data); // receive data from nrf24L01

        nunchuck_x_val = Nunchuck_Data[0]; // 20 - 220
        nunchuck_y_val = Nunchuck_Data[1]; // 31 - 227
        nunchuck_c_val = Nunchuck_Data[2] & 0x01; // 1 pressed, 0 not pressed
        nunchuck_z_val = (Nunchuck_Data[2] >> 1) & 0x01; //1 pressed, 0 not pressed
        
        printf("Nunchuck X Value: %u\r\n", Nunchuck_Data[0]);
        printf("Nunchuck Y Value: %u\r\n", Nunchuck_Data[1]);
        printf("C Val: %u\r\n", nunchuck_c_val);
        printf("Z Val: %u\r\n", nunchuck_z_val);
        printf("Target Wheel Speed: %d\r\n", target_speed);
        printf("Current Wheel Speed: %d\r\n", drive_speed);
        printf("Direction: %d\r\n", target_forward);
        
    }
    // Reset all values if not receiving anuything
    else{
        nunchuck_y_val = 128;
        nunchuck_x_val = 128;
        target_speed = 0;
    }

    // SERVO STUFF __________________________________________________________
    // deadzone so there isn't constant twitching around center position
    if(abs((int)nunchuck_x_val - 125) < 10) {
      nunchuck_x_val = 125;
      servo_pulse_us = SERVO_CENTER;
    }

    // Left
    else if (nunchuck_x_val > 125){
        servo_pulse_us = map(nunchuck_x_val, 125, 220, SERVO_CENTER, SERVO_MAX);
    }
    // Right
    else{
        servo_pulse_us = map(nunchuck_x_val, 125, 20, SERVO_CENTER, SERVO_MIN);
    }

    //printf("Current pulse width: %u us\r\n", servo_pulse_us);
    TIM5->CCR1 = servo_pulse_us; // servo pulse width

    // Servo testing
    //(servo_state) ? TIM5->CCR1 = 600 : TIM5->CCR1 = 2400;
    //servo_state = !servo_state;
    
    // BACK WHEEL MOTOR STUFF ____________________________________________________
    // Forward
    if(nunchuck_y_val > 142) {
        raw_speed = map(nunchuck_y_val, 142, 227, 255, MAX_SPEED);
        target_speed = 255 - raw_speed;

        ramp_speed = ACTIVE_RAMP;
    } 
    // Backwards
    else if(nunchuck_y_val < 118) {
        raw_speed = map(nunchuck_y_val, 118, 31, 255, MAX_SPEED);
        target_speed = -255 + raw_speed;

        ramp_speed = ACTIVE_RAMP;
    } 

    // Not moving
    else {
        target_speed = 0;  // deadzone
        ramp_speed = PASSIVE_RAMP;

    }
    
    // Correcting drive speed to target speed everyloop based on RAMP_SPEED
    if (drive_speed < target_speed){
        drive_speed += ramp_speed;
        
        // In case drive_speed overshoots target_speed
        if (drive_speed > target_speed){
            drive_speed = target_speed;
        }
        
    }

    // Vice-versa case
    else if (drive_speed > target_speed){
        drive_speed -= ramp_speed;

        if (drive_speed < target_speed){
            drive_speed = target_speed;
        }
    }
    // Direction logic

    // FORWARD
    if (drive_speed > 0)
    {
        digitalWrite(PA6, HIGH);
        target_forward = true;
    }

    // REVERSE
    else if (drive_speed < 0)
    {
        digitalWrite(PA6, LOW);
        target_forward = false;
    }

    // Reverse cmd since 255 = stopped, 0 = fullspeed on motor
    pwm_send = 255 - abs(drive_speed);
    analogWrite(PA5, constrain(pwm_send, 100, 255));
  }

}




// Back motor test stuff 
/*
void loop() {
    if (HAL_GetTick() - last_nrf_time >= 20) {
        last_nrf_time = HAL_GetTick();
        
        if (isDataAvailable(1) == 1) {
            NRF24_Receive(Nunchuck_Data);
        }
        
        nunchuck_y_val = Nunchuck_Data[1];
        
        printf("Y: %u\r\n", nunchuck_y_val);
        
        // direct mapping, no ramping
            if(nunchuck_y_val > 138) {
            digitalWrite(PA6, LOW);
            analogWrite(PA5, 180);
            printf("FORWARD\r\n");
        } else if(nunchuck_y_val < 118) {
            digitalWrite(PA6, HIGH);  
            analogWrite(PA5, 180);
            printf("REVERSE\r\n");
        } else {
            analogWrite(PA5, 255);
            printf("STOP\r\n");
}
    }
}
    */



void calibrateBike()
{
    printf("Hold bike at BALANCE POINT! Lock in!\r\n");
    HAL_Delay(500);

    printf("3\r\n");
    HAL_Delay(500);
    printf("2\r\n");

    HAL_Delay(500);
    printf("1\r\n");
    HAL_Delay(500);
    printf("CALIBRATING - KEEP STILL\r\n");

    g_gx_offset = 0.0f;
    g_gy_offset = 0.0f;
    g_gz_offset = 0.0f;

    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;
    float sum_ay = 0.0f;
    float sum_az = 0.0f;

    int valid_samples = 0;
    const int samples = 500;

    // Sampling 500 gyro and accelerometer offsets 
    for (int i = 0; i < samples; i++)
    {
        ICM20948_Data sample;
        if (ICM20948_Read(&sample))
        {
            sum_gx += sample.gyro_x;
            sum_gy += sample.gyro_y;
            sum_gz += sample.gyro_z;

            sum_ay += sample.accel_y;
            sum_az += sample.accel_z;

            valid_samples++;
        }
        HAL_Delay(4);
    }

    if (valid_samples == 0)
    {
        printf("CALIBRATION FAILED\r\n");
        return;
    }

    // Averaging the offset
    g_gx_offset = sum_gx / valid_samples;
    g_gy_offset = sum_gy / valid_samples;
    g_gz_offset = sum_gz / valid_samples;

    float avg_ay = sum_ay / valid_samples;
    float avg_az = sum_az / valid_samples;

    Balance_Init(avg_ay, avg_az);
    printf("Calibration done!!\r\n");
}

void CheckEmergencyStop()
{
    // Non-blocking check for one received UART character
    if (HAL_UART_Receive(&huart2, &uart_rx_char, 1, 0) == HAL_OK)
    {
        // x to diable motor
        if (uart_rx_char == 'x' || uart_rx_char == 'X')
        {
            emergency_stop = true;

            u = 0.0f;
            motor.move(0.0f);
            motor.disable();

            printf("\r\nEMERGENCY STOP - MOTOR DISABLED\r\n");
            printf("R to re-enable\r\n");
        }

        // r to renable motor
        else if (uart_rx_char == 'r' || uart_rx_char == 'R')
        {
            if (emergency_stop)
            {
                u = 0.0f;
                motor.enable();
                emergency_stop = false;
                printf("\r\nMotor re-enabled\r\n");
            }
        }
    }
}