#ifndef I2C_H
#define I2C_H

typedef struct {
    int16_t x_acc;
    int16_t y_acc;
    int16_t z_acc;
} imu_gyro_measurement;

extern imu_gyro_measurement gyro_data;

void init_i2c_imu(void);
void read_imu_gyro(void);
int16_t read_imu(void);

#endif