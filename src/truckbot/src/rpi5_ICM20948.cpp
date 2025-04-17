

#include "truckbot/rpi5_ICM20948.h"
#include <iostream>
#include <lgpio.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <cstdint>
#include <fcntl.h>
#include <algorithm>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
 
rpi5_ICM20948::rpi5_ICM20948(const uint8_t device) {
    uint8_t reg;
    handle = lgI2cOpen(1, device, 0);
    if (handle < 0) {
        fprintf(stderr, "Error opening IMU\n");
    }     

    resetMaster();

    // wake up device
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    reg = lgI2cReadByteData(handle, PWR_MGMT_1);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, PWR_MGMT_1, reg & 0b10111111);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));

    // select the second register bank
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x20);

    // set accel scale to +-4g
    lgI2cWriteByteData(handle, ACCEL_CONFIG, 0x1B);
    accelScale = 4.0f;
 
    // set gyro scale to +-500dps
    lgI2cWriteByteData(handle, GYRO_CONFIG_1, 0x1B);    
    gyroScale = 500.0f;

    // enable bypass mode
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, INT_PIN_CFG, 0x02);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));

    // set to be stop between reads, clock to 345.60khz
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x30);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, I2C_MST_CTRL, 0x17);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));

    // enable i2c master
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    reg = lgI2cReadByteData(handle, USER_CTRL);
    lgI2cWriteByteData(handle, USER_CTRL, reg | 0b00100000);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));

    // verify id
    uint8_t whoAmI = lgI2cReadByteData(handle, WHO_AM_I);
    if (whoAmI != DEVICE_ID)
    {
        fprintf(stderr, "Device ID wrong for icm20948\n");
    }

    // set mag to poweroff, sleep, then set to continuious mode 1
    writeMagReg(MAG_CONTROL_2, 0x00);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    writeMagReg(MAG_CONTROL_2, 0x02);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));

    // verify mag ID
    whoAmI = readMagReg(MAG_WHO_AM_I);
    if (whoAmI != MAG_DEVICE_ID)
    {
        fprintf(stderr, "Device ID wrong for mag");
    }
    
    // set up slave 0 to read into slave 0 data registers
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x30);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, I2C_SLV0_ADDR, 0x8C);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, I2C_SLV0_REG, 0x11);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, I2C_SLV0_CTRL, 0x89);

    usleep(10000);

    calibrateSensor();
}

void rpi5_ICM20948::resetMaster() {
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, PWR_MGMT_1, 0x80);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    uint8_t reset = lgI2cReadByteData(handle, PWR_MGMT_1);
    while (reset & 0b10000000) {
        usleep(5);
        reset = lgI2cReadByteData(handle, PWR_MGMT_1);
    }
}

uint8_t rpi5_ICM20948::readMagReg(uint8_t reg) { 
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x30);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, I2C_SLV4_ADDR, 0x8C);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, I2C_SLV4_REG, reg);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, I2C_SLV4_CTRL, 0x80);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    uint8_t reset = lgI2cReadByteData(handle, I2C_MST_STATUS);
    while (reset & 0b00100000) {
        usleep(1000);
        reset = lgI2cReadByteData(handle, I2C_MST_STATUS);
    }

    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x30);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    return lgI2cReadByteData(handle, I2C_SLV4_DI);
}

void rpi5_ICM20948::writeMagReg(uint8_t reg, uint8_t data) {
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x30);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, I2C_SLV4_ADDR, 0x0C);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, I2C_SLV4_REG, reg);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, I2C_SLV4_DO, data);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, I2C_SLV4_CTRL, 0x80);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);
    std::this_thread::sleep_for(std::chrono::microseconds(duration));
    uint8_t reset = lgI2cReadByteData(handle, I2C_MST_STATUS);
    while (reset & 0b00100000) {
        std::this_thread::sleep_for(std::chrono::microseconds(duration));
        reset = lgI2cReadByteData(handle, I2C_MST_STATUS);
    }
}

int rpi5_ICM20948::getMagnetometerData(float &ux, float &uy, float &uz) {
    static char data[8];
    int16_t uiUx, uiUy, uiUz;

    lgI2cWriteByteData(handle, REG_BANK_SEL, 0x00);
    int success = lgI2cReadI2CBlockData(handle, 0x3B, data, 8);
    
    uiUx = ((int16_t) data[1] << 8) | data[0];
    uiUy = ((int16_t) data[3] << 8) | data[2];
    uiUz = ((int16_t) data[5] << 8) | data[4];

    ux = ((uiUx / 20.0f) * 3.0f) - magOffset.x();
    uy = ((uiUy / 20.0f) * 3.0f) - magOffset.y();
    uz = ((uiUz / 20.0f) * 3.0f) - magOffset.z();

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
    ax = (uiAx / (32768.0f / accelScale) * G2MPSS) - accelOffset.x();
    ay = (uiAy / (32768.0f / accelScale) * G2MPSS) - accelOffset.y();
    az = (uiAz / (32768.0f / accelScale) * G2MPSS) - accelOffset.z();
    gx = (uiGx / (32768.0f / gyroScale) * DEG2RAD) - gyroOffset.x();
    gy = (uiGy / (32768.0f / gyroScale) * DEG2RAD) - gyroOffset.y();
    gz = (uiGz / (32768.0f / gyroScale) * DEG2RAD) - gyroOffset.z();

    return success;
}

void rpi5_ICM20948::calibrateSensor() {
    Eigen::Vector3f accelSum = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyroSum = Eigen::Vector3f::Zero();
    Eigen::Vector3f magMin = Eigen::Vector3f::Constant(1e9);
    Eigen::Vector3f magMax = Eigen::Vector3f::Constant(-1e9);

    float ax, ay, az, gx, gy, gz, ux, uy, uz;
    std::cout << "Starting calibration..." << std::endl;
    for (int i = 0; i < calibrateSamples; ++i) {
        getAccelerometerAndGyroscopeData(ax, ay, az, gx, gy, gz);
        getMagnetometerData(ux, uy, uz);

        Eigen::Vector3f accel(ax, ay, az);
        Eigen::Vector3f gyro(gx, gy, gz);
        Eigen::Vector3f mag(ux, uy, uz);

        accelSum += accel;
        gyroSum += gyro;
        magMin = magMin.cwiseMin(mag);
        magMax = magMax.cwiseMax(mag);

        usleep(125000); // Sleep for 125ms
    }
    std::cout << "Finished calibration" << std::endl;

    accelOffset = accelSum / calibrateSamples;
    gyroOffset = gyroSum / calibrateSamples;
    magOffset = (magMax + magMin) * 0.5f;
    
}

