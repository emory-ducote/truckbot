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
                         const double Kp = 1.25,
                         const double Ki = 0.0,
                         const double Kd = 0.0)
                         : chip(chip), 
                           pinOne(pinOne), 
                           pinTwo(pinTwo), 
                           Kp(Kp),
                           Ki(Ki),
                           Kd(Kd),
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

    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, pinOne, 0);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, pinTwo, 0);
}

MotorDriver::~MotorDriver()
{
    setMotorSpeed(0.0, false);
    usleep(10000);
    lgGpiochipClose(handle);
}

void MotorDriver::setMotorSpeed(const double targetSpeed, const bool usePID)
{
    double speed = targetSpeed;
    if (usePID)
    {
        double curTime = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
        double dt = curTime - previousCalculationStamp;
        double errorSpeed = targetSpeed - measuredSpeed.load();
        integralSpeed += errorSpeed * dt;
        double derivativeSpeed = (errorSpeed - prevErrorSpeed) / dt;
        double speed = Kp * errorSpeed + 
                       Ki * integralSpeed + 
                       Kd * derivativeSpeed;
        prevErrorSpeed = errorSpeed; 
    }

    double PWM = speedToPWM(speed);
    if (PWM > 0) 
    {
        lgTxPwm(handle, pinOne, 100, 0, 0, 0);
        lgTxPwm(handle, pinTwo, 100, PWM, 0, 0);

    }
    else if (PWM < 0)
    {
        lgTxPwm(handle, pinOne, 100, PWM, 0, 0);
        lgTxPwm(handle, pinTwo, 100, 0, 0, 0);

    }
    else 
    {
        lgTxPwm(handle, pinOne, 100, 0, 0, 0);
        lgTxPwm(handle, pinTwo, 100, 0, 0, 0);
    }
    
}

double MotorDriver::speedToPWM(const double speed) 
{
    // needs to be < 100%
    double lower = -0.99; 
    double upper = 0.99;
    
    return std::abs(std::max(lower, std::min(speed, upper)) * 100.0);
}

