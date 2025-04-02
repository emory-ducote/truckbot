#include "truckbot/ICM20948.h"
#include <iostream> 

int main() {


    ICM20948 imu(0,0,0,0); 
    while(true) {
        static float ax, ay, az, gx, gy, gz;
        imu.read_accel_gyro_g_dps(ax, ay, az, gx, gy, gz);
        std::cout << "ax: " << ax << std::endl;
        std::cout << "ay: " << ay << std::endl;
        std::cout << "az: " << az << std::endl;
        std::cout << "gx: " << gx << std::endl;
        std::cout << "gy: " << gy << std::endl;
        std::cout << "gz: " << gz << std::endl;
    }
    
    return 0;
  }