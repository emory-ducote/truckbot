

#include "truckbot/rpi5_ICM20948.h"
#include <iostream>
#include <lgpio.h>
#include <unistd.h>
#include <stdio.h>
#include <cstdint>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
 
rpi5_ICM20948::rpi5_ICM20948(const uint8_t device) {
    handle = lgI2cOpen(1, device, 0);
    if (handle < 0) {
        fprintf(stderr, "Error opening IMU\n");
    }     

    // wake up device
    lgI2cWriteByteData(handle, PWR_MGMT_1, 0x00);
    usleep(10);

    // auto get clock source
    lgI2cWriteByteData(handle, PWR_MGMT_1, 0x01);

    // disable accel/gyro
    lgI2cWriteByteData(handle, PWR_MGMT_2, 0x3F);
    // enable accel/gyro
    lgI2cWriteByteData(handle, PWR_MGMT_2, 0x00);

    // check if everything is fucntioning
    uint8_t whoAmI = lgI2cReadByteData(handle, WHO_AM_I);
    if (whoAmI != DEVICE_ID)
    {
        fprintf(stderr, "Error initializing IMU");
    }
    
    // set accel scale to +-4g
    lgI2cWriteByteData(handle, BANK_2_SHIFT | ACCEL_CONFIG, 0x13);
    accelScale = 4.0;

    // set accel scale to 1.125 kHz/(1+10) ~ 100 Hz
    lgI2cWriteByteData(handle, BANK_2_SHIFT | ACCEL_SMPLRT_DIV_2, 0x0A);
 
    // set gyro scale to +-500dps
    lgI2cWriteByteData(handle, BANK_2_SHIFT | GYRO_CONFIG_1, 0x2B);
    gyroScale = 500.0;

    // set gyro scale to 1.1 kHz/(1+10) ~ 100 Hz
    lgI2cWriteByteData(handle, BANK_2_SHIFT | GYRO_SMPLRT_DIV, 0x0A);
    
     
}
 
int rpi5_ICM20948::readAccel(float &ax, float &ay, float &az) {
    static char data[6];
    
    // Read accel and gyroscope raw data in
    int success = lgI2cReadI2CBlockData(handle, ACCEL_XOUT_H, data, 6);
     
    ax = (float) (((int16_t) data[0] << 8) | data[1]) * accelScale/ 32768.0;
    ay = (float) (((int16_t) data[2] << 8) | data[3]) * accelScale/ 32768.0;
    az = (float) (((int16_t) data[4] << 8) | data[5]) * accelScale / 32768.0;
     
    return success;
}

int rpi5_ICM20948::readGyro(float &gx, float &gy, float &gz) {
    static char data[6];
    
    // Read accel and gyroscope raw data in
    int success = lgI2cReadI2CBlockData(handle, GYRO_XOUT_H, data, 6);
     
    gx = (float) (((int16_t) data[0] << 8) | data[1]) / 32768.0 * gyroScale;
    gy = (float) (((int16_t) data[2] << 8) | data[3]) / 32768.0 * gyroScale;
    gz = (float) (((int16_t) data[4] << 8) | data[5]) / 32768.0 * gyroScale;
     
    return success;
}

