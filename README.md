# ESP8266 RTOS SyncdServo

In this project the data read from a 6 axis motion sensor was used to guide a small servo motor. The goal was to replicate the angle measured by the sensor on the small arm attached to the servo. The servo and the sensor were attached to two different micro-controllers, the micro-controllers communicated through a wireless protocol.

## List of components

#### Sender:
- ESP8266-01
- MPU6050 attitude sensor
- 2x 4.7k ohm pull-up resistors
#### Receiver:
- ESP8266-01
- SG90 servo motor
- breadboard power module

## More details

The small library I wrote for MPU6050 is not complete and doesn't cover all the sensor functions, only those necessary for this project. 
For the communication between the two micro-controllers the ESPNOW protocol was chosen mainly for two reasons: First, the small amount of data that needed to be sent and received each time and second the uncentralized nature of the project.
This project was developed using the ESP RTOS SDK, it was chosen not to use the MPU DMP and instead compute the necessary data directly in the code.

## Usage and Requirements

This project requires the [ESP RTOS v3.3]. Follow their [instructions] to set up the toolchain and the SDK.   
`make app` compile the code.  
`make app flash` flash it into the ESP-01.  
Compile and flash sender and receiver separately.

## Sender Schematics
![Sender Schematics](SYNCDSENDER.png)

## Receiver Schematics
![Receiver Schematics](SYNCDRECEIVER.png)

[ESP RTOS v3.3]: <https://github.com/espressif/ESP8266_RTOS_SDK/tree/release/v3.3>
[instructions]: <https://github.com/espressif/ESP8266_RTOS_SDK/tree/release/v3.3#get-toolchain>
[ESP RTOS]: <https://github.com/espressif/ESP8266_RTOS_SDK/tree/release/v3.3#get-esp8266_rtos_sdk>
[Path]: <https://github.com/espressif/ESP8266_RTOS_SDK/tree/release/v3.3#setup-path-to-esp8266_rtos_sdk>

