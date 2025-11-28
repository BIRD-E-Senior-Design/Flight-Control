# BIRD E Flight Control

Drone flight controller firmware written for an RP2350a.
## Contents

## Relevant Diagrams
### Software Block Diagram
![Block Diagram](assets/SoftwareBlock.png)

### State Machine on Core 1
![State Machine](assets/StateMachine.png)

## Components List

- MCU: RP2350a
- Wifi & Video: Raspberry Pi Zero 2W
- IMU: MPU 9250 (used in testing for now) --> BNO055 9DoF
- Altimeter: MPU L35115A2
- Laser ToF Sensor:
- Remote Control: SPI bridge to Zero 2W
- Drift Cam:

## Datasheets

- [RP2350](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [PICO SDK](https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)
- [MPU 9250 IMU](https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU9250REV1.0.pdf)
- [BNO055 IMU](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
- [MPU L3115A2 Barometric Altimeter](https://cdn-shop.adafruit.com/datasheets/1893_datasheet.pdf)
