# Self-Balancing-Bike-

Self balancing bike that uses a flywheel to auto balance. Project drive folder down below (has materials, diagrams, etc.):

*Seperate folders for bike firmware and external wii-nunchuk controller firmware*

[Project Drive Folder](https://drive.google.com/drive/folders/1dX5lAlIIB4DTDAcDznrTppuNJ5s5Nt8N?usp=sharing)

# Quick demo video:

https://github.com/user-attachments/assets/d5414e99-e70b-4be9-8098-3ad1ea3dc3ff

  
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
