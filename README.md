

# Self-Balancing-Bike-

Self balancing bike that uses a flywheel to auto balance. Project drive folder down below (has materials, diagrams, etc.):

*Seperate folders for bike firmware and external wii-nunchuk controller firmware*

[Project Drive Folder](https://drive.google.com/drive/folders/1dX5lAlIIB4DTDAcDznrTppuNJ5s5Nt8N?usp=sharing)


## Mechanical Build

Very similar to RemRC's - shoutout to him for the mechanical build! I just have a different electrical system (different IMU, motor controller, driver, control system, etc.), motor, motor mount, and flywheel. BUT everything else is pretty similar to his build mechanically. I am also using SimpleFOC to run my bldc motor instead of using a Nidec

<img width="663" height="468" alt="image" src="https://github.com/user-attachments/assets/5b239078-1093-499a-bc1b-c7a09a87676a" />

## First video of bike actually balancing!
https://github.com/user-attachments/assets/1babaf89-0a4d-4dfa-8808-23e55a363219

Definitely need to clean up wiring AND figure out how to balance with the back motor on in the future though.

## Current State of the bike
Bike can balance without movement, but when driving forward/backward, vibrations occur which knocks off the loop (WIP) 

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

## Mechanical Build
- **Chassis:** Custom 3D-printed PLA frame (~35 cm long)
- **Reaction Wheel:** 3D-printed PLA flywheel (120 mm diameter)
- **Flywheel Weights:** 12 M4 screws (16 mm), each with 3 nuts (36 total)
- **Front Wheel:** LEGO parts 2902 and 2903
- **Rear Wheel:** LEGO tire 88516
- **Drive System:** 280 mm timing belt
- **Steering:** MG995 servo-actuated front fork
- **Assembly:** M2, M2.5, M3, and M4 nuts/bolts

# How the control loop works

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
