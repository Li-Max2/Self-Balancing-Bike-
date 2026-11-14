
https://github.com/user-attachments/assets/3e05bd81-1092-4b02-91d9-a487e8285292


# Self-Balancing-Bike-

Self balancing bike that uses a flywheel to auto balance. Project drive folder down below (has materials, diagrams, etc.):

*Seperate folders for bike firmware and external wii-nunchuk controller firmware*

## Electrical Build
- **Microcontroller:** STM32F411 Blackpill
- **IMU:** ICM-20948 (accelerometer + gyroscope)
- **Reaction Wheel Motor:** GM3506 BLDC motor
- **Encoder:** AS5048A magnetic encoder
- **Motor Driver:** DRV8313
- **Rear Wheel Motor:** Nidec 24H
- **Steering Servo:** MG995
- **Wireless Communication:** NRF24L01 transceiver
- **Remote Controller:** Wii Nunchuck
- **Power Supply:** 3S 11.1V LiPo battery (for now just using external PSU with battery disattached)


## Updated balance video - got it to balance with the motor and battery attatched!

(sorry for the bad video quality my phone camera is terrible)

https://github.com/user-attachments/assets/883d2daf-59d8-4193-acc7-02b97293eb2b

## Quick explanation of how the bike works
https://github.com/user-attachments/assets/564dc976-a3f2-43d4-baaa-4a28df0c77de


## Mechanical Build
- **Chassis:** Custom 3D-printed PLA frame (~35 cm long)
- **Reaction Wheel:** 3D-printed PLA flywheel (120 mm diameter)
- **Flywheel Weights:** 12 M4 screws (16 mm), each with 3 nuts (36 total)
- **Front Wheel:** LEGO parts 2902 and 2903
- **Rear Wheel:** LEGO tire 88516
- **Drive System:** 280 mm timing belt
- **Steering:** MG995 servo-actuated front fork
- **Assembly:** M2, M2.5, M3, and M4 nuts/bolta

Very similar to RemRC's - shoutout to him for the mechanical build! I just have a different electrical system (different IMU, motor controller, driver, control system, etc.), motor, motor mount, and flywheel. BUT everything else is pretty similar to his build mechanically. I am also using SimpleFOC to run my bldc motor instead of using a Nidec

<img width="663" height="468" alt="image" src="https://github.com/user-attachments/assets/5b239078-1093-499a-bc1b-c7a09a87676a" />

[Project Drive Folder](https://drive.google.com/drive/folders/1dX5lAlIIB4DTDAcDznrTppuNJ5s5Nt8N?usp=sharing)

## Current State of the bike
Bike can balance without movement, but when driving forward/backward, vibrations occur which knocks off the loop (WIP) 

## How the control loop works

1. ICM-20948 accelerometer AND gyroscope combine to derive an angle measurement.
- Atan2() converts the accelerometer y and z axis gravity readings into an angle away from the vertical axis
- gyroscope measures angular velocity (dtheta/dt) (deg/s), and is converted into rad/s. From there, you can predict the angle changed over some dt:

$\Delta\theta=\omega\Delta t$ 

- everytime the gyro updates, you can calculate a new angle:

$\theta_{\text{predicted}} =\theta_{\text{previous}}+\omega\Delta t$

TO BE CONTINUED


  PIN MAPPINGS:

Reaction wheel motor driver
- PA8 -> Phase A PWM
- PA9 -> Phase B PWM
- PA10 -> Phase C PWM
- PA7 -> Enable

AS5048A REACTION WHEEL ENCODER (SPI1)
- PB3 -> SCK
- PB4 -> MISO
- PB5 -> MOSI
- PB10 -> CS / nCS

ICM20948 IMU (I2C1)
- PB6 -> SCL
- PB7 -> SDA

NRF24L01 (SPI2)
- PB13 -> SCK
- PB14 -> MISO
- PB15 -> MOSI
- PB12 -> CSN
- PB0 -> CE

Back Nidec motor
- PA4 -> Start / Enable
- PA5 -> Speed PWM
- PA6 -> Direction

Back motor encoder
- PB8 -> Encoder input
- PB9 -> Encoder input

Steering servo
- PA0 -> Servo PWM

USART PRINT DEBUGGING
- PA2 -> TX
- PA3 -> RX

Status LED
- PC13 -> LED

First ever balance!
https://github.com/user-attachments/assets/1babaf89-0a4d-4dfa-8808-23e55a363219
## First video of bike actually balancing!

