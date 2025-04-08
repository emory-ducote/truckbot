

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
        fprintf(stderr, "Error opening IMU at %X\n");
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

    // check if everything is functioning
    uint8_t whoAmI = lgI2cReadByteData(handle, WHO_AM_I);
    if (whoAmI != DEVICE_ID)
    {
        fprintf(stderr, "Device ID wrong for icm20948\n");
    }
    
    // select the second register bank
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x20);

    // set accel scale to +-4g
    lgI2cWriteByteData(handle, ACCEL_CONFIG, 0x1B);
    accelScale = 4.0f;
 
    // set gyro scale to +-500dps
    lgI2cWriteByteData(handle, GYRO_CONFIG_1, 0x1B);    
    gyroScale = 500.0f;

    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);
    
    // enable the mag
    lgI2cWriteByteData(handle, INT_PIN_CFG, 0x02);

    whoAmI = readMagReg(MAG_WHO_AM_I);
    if (whoAmI != MAG_DEVICE_ID)
    {
        fprintf(stderr, "Device ID wrong for mag %x", (int) whoAmI);
    }

    writeMagReg(MAG_CONTROL_2, 0x08);
}

uint8_t rpi5_ICM20948::readMagReg(uint8_t reg) {
    uint8_t result;
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x30);
    lgI2cWriteByteData(handle, I2C_SLV0_ADDR, 0x0C | 0x80);
    lgI2cWriteByteData(handle, I2C_SLV0_REG, reg);
    lgI2cWriteByteData(handle, I2C_SLV0_CTRL, 0x80 | 0x01);
    usleep(10);
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);
    return lgI2cReadByteData(handle, 0x3B);
}

void rpi5_ICM20948::writeMagReg(uint8_t reg, uint8_t data) {
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x30);
    lgI2cWriteByteData(handle, I2C_SLV0_ADDR, 0x0C);
    lgI2cWriteByteData(handle, I2C_SLV0_REG, reg);
    lgI2cWriteByteData(handle, I2C_SLV0_DO, data);
    lgI2cWriteByteData(handle, I2C_SLV0_CTRL, 0x80 | 0x01);
    usleep(10);

}

int rpi5_ICM20948::getMagnetometerData(float &ux, float &uy, float &uz) {
    static char data[8];
    int16_t uiUx, uiUy, uiUz;

    //case 0
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x30);
    lgI2cWriteByteData(handle, I2C_SLV0_ADDR, 0x0C | 0x80);
    lgI2cWriteByteData(handle, I2C_SLV0_REG, MAG_STATUS_1);
    lgI2cWriteByteData(handle, I2C_SLV0_CTRL, 0x80 | 1);
    // usleep(100);
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);

    //case 2
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x30);
    lgI2cWriteByteData(handle, I2C_SLV0_ADDR, 0x0C | 0x80);
    lgI2cWriteByteData(handle, I2C_SLV0_REG, MAG_XOUT_L);
    lgI2cWriteByteData(handle, I2C_SLV0_CTRL, 0x80 | 6);
    // usleep(100);
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);

    //case 3
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x30);
    lgI2cWriteByteData(handle, I2C_SLV0_ADDR, 0x0C | 0x80);
    lgI2cWriteByteData(handle, I2C_SLV0_REG, MAG_XOUT_L);
    lgI2cWriteByteData(handle, I2C_SLV0_CTRL, 0x80 | 1);
    // usleep(100);
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);
    int success = lgI2cReadI2CBlockData(handle, 0x3B, data, 8);
    // std::cout << std::hex << (int) (data[0] & 0x01) << std::endl;

    uiUx = ((int16_t) data[1] << 8) | data[0];
    uiUy = ((int16_t) data[3] << 8) | data[2];
    uiUz = ((int16_t) data[5] << 8) | data[4];

    ux = (uiUx / 20.0f) * 3.0f;
    uy = (uiUy / 20.0f) * 3.0f;
    uz = (uiUz / 20.0f) * 3.0f;

    //case 4
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x30);
    lgI2cWriteByteData(handle, I2C_SLV0_ADDR, 0x0C | 0x80);
    lgI2cWriteByteData(handle, I2C_SLV0_REG, MAG_STATUS_2);
    lgI2cWriteByteData(handle, I2C_SLV0_CTRL, 0x80 | 1);
    // usleep(100);
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);

    return success;
}
 
int rpi5_ICM20948::getAccelerometerAndGyroscopeData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    static char data[12];
    int16_t uiAx, uiAy, uiAz, uiGx, uiGy, uiGz;
    
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);
    
    // Read accel and gyroscope raw data in
    int success = lgI2cReadI2CBlockData(handle, ACCEL_XOUT_H, data, 12);
     
    uiAx = (int16_t) (data[0] << 8) | data[1];
    uiAy = (int16_t) (data[2] << 8) | data[3];
    uiAz = (int16_t) (data[4] << 8) | data[5];
    uiGx = (int16_t) (data[6] << 8) | data[7];
    uiGy = (int16_t) (data[8] << 8) | data[9];
    uiGz = (int16_t) (data[10] << 8) | data[11];
    ax = uiAx / (32768.0f / accelScale) * G2MPSS;
    ay = uiAy / (32768.0f / accelScale) * G2MPSS;
    az = uiAz / (32768.0f / accelScale) * G2MPSS;
    gx = uiGx / (32768.0f / gyroScale) * DEG2RAD;
    gy = uiGy / (32768.0f / gyroScale) * DEG2RAD;
    gz = uiGz / (32768.0f / gyroScale) * DEG2RAD;

    return success;
}

