# Self-Balancing-Bike-

Self balancing bike that uses a flywheel to auto balance. Project drive folder down below (has materials, diagrams, etc.):

[Project Drive Folder](https://drive.google.com/drive/folders/1dX5lAlIIB4DTDAcDznrTppuNJ5s5Nt8N?usp=sharing)

<img width="2048" height="1536" alt="570" src="https://github.com/user-attachments/assets/a4ee3c80-eab1-4696-b635-d4557c42a267" />

PIN MAPPINGS:

Reaction wheel motor driver
- PA8 -> Phase A PWM
- PA9 -> Phase B PWM
- PA10 -> Phase C PWM
(CAN DO 10-9-8 DIRECTION ASWELL)
- PA7 -> Enable

AS5600 ENCODER
- PB6 -> SCL
- PB7 -> SDA

ICM20948 IMU
- PB10 -> SCL
- PB3 -> SDA

NRF24L01
- PB13 -> SCK
- PB14 -MISO
- PB15 -> MOSI
- PB12 -> CSN
- PB4 ->CE

Back Nidec motor (for moving)
- PA4 -> Start/Enable
- PA5 -> Speed PWM
- PA6 -> Direction

Steering servo
- PA0 -> Servo PWM

USART PRINT DEBUGGING
- PA2 -> TX
- PA3 -> RX

Will be updating this part soon!
