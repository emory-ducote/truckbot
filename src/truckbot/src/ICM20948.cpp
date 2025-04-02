/***************************************************************************//**
 * @file ICM20948.cpp
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

 #include "truckbot/ICM20948.h"
 #include <iostream>
 #include <lgpio.h>
 #include <unistd.h>
 #include <stdio.h>
 #include <cstdint>
 #include <fcntl.h>
 #include <sys/ioctl.h>
 #include <linux/i2c-dev.h>
 
 ICM20948::ICM20948(int16_t *gyroOffset_1000dps_xyz, int16_t *accelOffset_32g_xyz, float *magOffset_xyz, float *magScale_xyz) {
     
     handle = lgI2cOpen(1, 0x68, 0);
     std::cout << handle << std::endl;
     lgI2cWriteByteData(handle, ICM20948_REG_PWR_MGMT_1, ICM20948_BIT_CLK_PLL);
     usleep(10);
     
     /* Configure accelerometer */
     set_accel_bandwidth(ICM20948_ACCEL_BW_1210HZ);
     set_accel_fullscale(ICM20948_ACCEL_FULLSCALE_8G);
     
     /* Configure gyroscope */
     set_gyro_bandwidth(ICM20948_GYRO_BW_12100HZ);
     set_gyro_fullscale(ICM20948_GYRO_FULLSCALE_1000DPS);
     
 }
 
 uint32_t ICM20948::set_gyro_fullscale(uint8_t gyroFs) {
     uint8_t reg;
     
     /* Calculate and save gyro resolution */
     switch ( gyroFs ) {
         case ICM20948_GYRO_FULLSCALE_250DPS:
             m_gyroRes = 250.0f / 32768.0f;
             m_gyroRes_rad = m_gyroRes / RAD2DEG;
             break;
         case ICM20948_GYRO_FULLSCALE_500DPS:
             m_gyroRes = 500.0f / 32768.0f;
             m_gyroRes_rad = m_gyroRes / RAD2DEG;
             break;
         case ICM20948_GYRO_FULLSCALE_1000DPS:
             m_gyroRes = 1000.0f / 32768.0f;
             m_gyroRes_rad = m_gyroRes / RAD2DEG;
             break;
         case ICM20948_GYRO_FULLSCALE_2000DPS:
             m_gyroRes = 2000.0f / 32768.0f;
             m_gyroRes_rad = m_gyroRes / RAD2DEG;
             break;
         default:
             return ERROR;
     }
         
     gyroFs &= ICM20948_MASK_GYRO_FULLSCALE;
     reg = lgI2cReadByteData(handle, ICM20948_REG_GYRO_CONFIG_1);
     reg &= ~(ICM20948_MASK_GYRO_FULLSCALE);
     reg |= gyroFs;
     lgI2cWriteByteData(handle, ICM20948_REG_GYRO_CONFIG_1, reg);
 
     return 0;
 }
 
 uint32_t ICM20948::set_accel_fullscale(uint8_t accelFs) {
     uint8_t reg;
 
     /* Calculate and save accel resolution */
     switch ( accelFs ) {
         case ICM20948_ACCEL_FULLSCALE_2G:
             m_accelRes = 2.0f / 32768.0f;
             break;
         case ICM20948_ACCEL_FULLSCALE_4G:
             m_accelRes = 4.0f / 32768.0f;
             break;
         case ICM20948_ACCEL_FULLSCALE_8G:
             m_accelRes = 8.0f / 32768.0f;
             break;
         case ICM20948_ACCEL_FULLSCALE_16G:
             m_accelRes = 16.0f / 32768.0f;
             break;
         default:
             return ERROR;
     }
         
     accelFs &= ICM20948_MASK_ACCEL_FULLSCALE;
     reg = lgI2cReadByteData(handle, ICM20948_REG_ACCEL_CONFIG);
     reg &= ~(ICM20948_MASK_ACCEL_FULLSCALE);
     reg |= accelFs;
     lgI2cWriteByteData(handle, ICM20948_REG_ACCEL_CONFIG, reg);
     
     /*  Acceleration of gravity in LSB  */
     // m_g = (int16_t) (1 / m_accelRes + 0.5);
 
     return 0;
 }
 
 
 bool ICM20948::read_accel_gyro(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
     static char data[12];
     
     /* Read accelerometer and gyroscope raw data into a data array */
     lgI2cReadI2CBlockData(handle, ICM20948_REG_ACCEL_XOUT_H_SH, data, 12);
     
     /* Convert the MSB and LSB into a signed 16-bit value */
     ax = ((int16_t) data[0] << 8) | data[1];
     ay = ((int16_t) data[2] << 8) | data[3];
     az = ((int16_t) data[4] << 8) | data[5];
     gx = ((int16_t) data[6] << 8) | data[7];
     gy = ((int16_t) data[8] << 8) | data[9];
     gz = ((int16_t) data[10] << 8) | data[11];
     
     return true;
 }
 
 bool ICM20948::read_accel_gyro_g_dps(float &ax_g, float &ay_g, float &az_g, float &gx_dps, float &gy_dps, float &gz_dps) {
     static int16_t ax, ay, az, gx, gy, gz;
     
     read_accel_gyro(ax, ay, az, gx, gy, gz);
 
     /* Multiply the accelerometer values with their resolution to transform them into g */
     ax_g = (float) ax * m_accelRes;
     ay_g = (float) ay * m_accelRes;
     az_g = (float) az * m_accelRes;
     
     /* Multiply the gyroscope values with their resolution to transform them into deg/s */
     gx_dps = (float) gx * m_gyroRes;
     gy_dps = (float) gy * m_gyroRes;
     gz_dps = (float) gz * m_gyroRes;
     
     return true;
 }
 
 uint32_t ICM20948::set_accel_bandwidth(uint8_t accelBw) {
     uint8_t reg;
 
     /* Read GYRO_CONFIG_1 register */
     reg = lgI2cReadByteData(handle, ICM20948_REG_ACCEL_CONFIG);
     reg &= ~(ICM20948_MASK_ACCEL_BW);
 
     /* Write new bandwidth value to gyro config register */
     reg |= (accelBw & ICM20948_MASK_ACCEL_BW);
     lgI2cWriteByteData(handle, ICM20948_REG_ACCEL_CONFIG, reg);
 
     return 0;
 }
 
 uint32_t ICM20948::set_gyro_bandwidth(uint8_t gyroBw) {
     uint8_t reg;
 
     /* Read GYRO_CONFIG_1 register */
     reg = lgI2cReadByteData(handle, ICM20948_REG_GYRO_CONFIG_1);
     reg &= ~(ICM20948_MASK_GYRO_BW);
 
     /* Write new bandwidth value to gyro config register */
     reg |= (gyroBw & ICM20948_MASK_GYRO_BW);
     lgI2cWriteByteData(handle, ICM20948_REG_GYRO_CONFIG_1, reg);
 
     return OK;
 }