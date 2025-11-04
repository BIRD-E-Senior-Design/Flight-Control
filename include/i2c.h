#ifndef I2C_H
#define I2C_H

//IMU measurement type
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} imu_measurement;

extern imu_measurement gyro_data;
extern imu_measurement accel_data;
extern imu_measurement magnet_data;

void init_i2c0(void); //initialize i2c0 for IMU

//IMU read functions
void read_imu_gyroscope(void);
void read_imu_accelerometer(void);
void read_imu_magnetometer(void);

int16_t read_imu(void); //demo

#endif