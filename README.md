# ESP8266 RTOS SyncdServo

In this project the data read from a 6 axis motion sensor was used to guide a small servo motor. The goal was to replicate the angle measured by the sensor with the small arm attached to the servo. The servo and the sensor were attached to two different micro-controllers, and the micro-controllers communicated using a wireless protocol.
I went into this project full aware of its limitations, the most important being the unreliability of the position data calculated only using 3DOF acceleration and 3DOF gyroscope sensors. Under this aspect the MPU6050, after being properly tuned, proved to be a very good sensor and the drift in the data, while always present, was partialy mitigated.

## List of components

#### Sender:
- esp8266-01
- mpu6050 attitude sensor
- 2x 4.7k ohm pull-up resistors
#### Receiver:
- esp8266-01
- sg90 servo motor
- breadboard power module

## Details

For the communication between the two esps the espnow protocol was used for primarly two reasons: First, the small amount of data that needed to be sent and received each time and second the un-centralized nature of the project.
Both the esps uses the esp-rtos and no external mpu6050 related library has been used to communicate with the mpu6050, also it was chasen not to use the MPU DMP, the math involved in the project was coded by hand.

## Sender Schematics
![Sender Schematics](SYNCDSENDER.png)

## Receiver Schematics
![Receiver Schematics](SYNCDRECEIVER.png)
