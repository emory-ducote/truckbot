#include "MotorDriver.h"
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <algorithm>
#include <thread>
#include <chrono>

MotorDriver::MotorDriver(const uint8_t& chip,
                         const uint8_t& pinOne,
                         const uint8_t& pinTwo,
                         const double Kp,
                         const double Ki,
                         const double Kd,
                         const double deadband)
                         : chip(chip),
                           pinOne(pinOne),
                           pinTwo(pinTwo),
                           Kp(Kp),
                           Ki(Ki),
                           Kd(Kd),
                           deadband(deadband),
                           measuredSpeed(0.0),
                           prevErrorSpeed(0.0),
                           integralSpeed(0.0)
{ 
    previousCalculationStamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
    handle = lgGpiochipOpen(chip);
    if (handle < 0) {
        fprintf(stderr, "Error opening GPIO chip %d\n", chip);
    }
    std::cout << "PIN: " << pinOne << "," << pinTwo << std::endl;
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, pinOne, 0);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, pinTwo, 0);

    setTargetSpeed(0.0);
    updateMotorSpeed(true);
}

MotorDriver::~MotorDriver()
{
    setTargetSpeed(0.0);
    updateMotorSpeed(false);
    usleep(10000);
    lgGpioWrite(handle, pinOne, 0);
    lgGpioWrite(handle, pinTwo, 0);
    lgGpiochipClose(handle);
}

void MotorDriver::updateMotorSpeed(const bool usePID)
{
    double target = targetSpeed.load();

    double curTime = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
    double dt = curTime - previousCalculationStamp;
    previousCalculationStamp = curTime;

    double speed = target;
    if (usePID)
    {
        if (target == 0.0)
        {
            integralSpeed = 0.0;
            prevErrorSpeed = 0.0;
        }
        else
        {
            double error = target - measuredSpeed.load();
            if (std::abs(error) < deadband)
            {
                prevErrorSpeed = error;
                return;
            }
            integralSpeed = std::clamp(integralSpeed + error * dt, -1.0, 1.0);
            double derivative = (error - prevErrorSpeed) / dt;
            speed = Kp * error + Ki * integralSpeed + Kd * derivative;
            prevErrorSpeed = error;
        }
    }

    double duty = speedToPWM(speed);
    double dutyOne = 0.0;
    double dutyTwo = 0.0;
    if (speed < 0) { dutyOne = duty; }
    if (speed > 0) { dutyTwo = duty; }
    lgTxPwm(handle, pinOne, 100, dutyOne, 0, 0);
    lgTxPwm(handle, pinTwo, 100, dutyTwo, 0, 0);
}

double MotorDriver::speedToPWM(const double speed)
{
    return std::abs(std::clamp(speed, -0.99, 0.99) * 100.0);
}

