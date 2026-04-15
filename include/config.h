#ifndef CONFIG_H
#define CONFIG_H

//  LOGGING
    #define LOG_MODE_0
    #define LOG_MODE_1

//  PINS
    #define PIN_I2C1_SDA 18 //I2C Buses
    #define PIN_I2C1_SCL 19
    #define PIN_I2C0_SDA 28
    #define PIN_I2C0_SCL 29

    #define PIN_IMU_HARD_RESET 22 //IMU
    #define PIN_IMU_RESET 23
    #define PIN_IMU_INT 26
    #define PIN_IMU_ADR 17
    #define PIN_IMU_PS0 2
    #define PIN_IMU_PS1 3
    #define PIN_IMU_STATUS_LED 9

    #define PIN_TOF_I2C_RST 24 //ToF
    #define PIN_TOF_LPN 25
    #define PIN_TOF_INT 27
    #define PIN_TOF_STATUS_LED 8

    #define PIN_ALT_INT1 20 //Altimeter
    #define PIN_ALT_INT2 21
    #define PIN_ALT_STATUS_LED 11

    #define PIN_DRIFT_TX 12 //Optical Flow Cam
    #define PIN_DRIFT_RX 15
    #define PIN_DRIFT_CSN 13
    #define PIN_DRIFT_SCLK 14
    #define PIN_DRIFT_INT 16
    #define PIN_DRIFT_STATUS_LED 10

    #define PIN_MOTOR1 32 //Motors
    #define PIN_MOTOR2 33
    #define PIN_MOTOR3 34
    #define PIN_MOTOR4 35
    #define PIN_MOTOR1_STATUS_LED 4
    #define PIN_MOTOR2_STATUS_LED 5
    #define PIN_MOTOR3_STATUS_LED 6
    #define PIN_MOTOR4_STATUS_LED 7    

    #define PIN_RPZ_TX 37 //Raspberry Pi Zero 2W
    #define PIN_RPZ_RX 36

#endif