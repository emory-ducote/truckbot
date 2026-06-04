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
    updateMotorSpeed(true);
    usleep(10000);
    lgGpiochipClose(handle);
}

void MotorDriver::updateMotorSpeed(const bool usePID)
{
    double target = targetSpeed.load();

    // When commanded to stop, cut power immediately and reset PID state.
    double curTime = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();
        double dt = curTime - previousCalculationStamp;
        previousCalculationStamp = curTime;

    double speed = target;
    if (usePID)
    {
        double errorSpeed = target - measuredSpeed.load();
        if (std::abs(errorSpeed) < deadband)
        {
            prevErrorSpeed = errorSpeed;
            return;
        }
        integralSpeed += errorSpeed * dt;
        const double integralLimit = 1.0;
        integralSpeed = std::clamp(
            integralSpeed,
            -integralLimit,
            integralLimit
        );
        double derivativeSpeed = (errorSpeed - prevErrorSpeed) / dt;
        speed = Kp * errorSpeed + 
                       Ki * integralSpeed + 
                       Kd * derivativeSpeed;
        prevErrorSpeed = errorSpeed; 
    }

    double PWM = speedToPWM(speed);
    if (speed > 0) 
    {
        lgTxPwm(handle, pinOne, 100, 0, 0, 0);
        lgTxPwm(handle, pinTwo, 100, PWM, 0, 0);

    }
    else if (speed < 0)
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

