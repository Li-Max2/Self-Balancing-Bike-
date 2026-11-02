

# Self-Balancing-Bike-

Self balancing bike that uses a flywheel to auto balance. Project drive folder down below (has materials, diagrams, etc.):

*Seperate folders for bike firmware and external wii-nunchuk controller firmware*

[Project Drive Folder](https://drive.google.com/drive/folders/1dX5lAlIIB4DTDAcDznrTppuNJ5s5Nt8N?usp=sharing)

# Quick demo video of bike balancing while stationary with back wheel motor disabled (sorry for the poor phone camera quality lol):

https://github.com/user-attachments/assets/d5414e99-e70b-4be9-8098-3ad1ea3dc3ff

# Mechanical Build
<img width="663" height="468" alt="image" src="https://github.com/user-attachments/assets/5b239078-1093-499a-bc1b-c7a09a87676a" />

# Current State of the bike
Bike can balance without movement, but when driving forward/backward, vibrations occur which knocks off the loop (WIP) 

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
