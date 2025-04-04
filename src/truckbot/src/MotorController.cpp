#include <cmath>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include "truckbot/MotorController.h"

MotorController::MotorController(const uint8_t& chip, const uint8_t& leftOne, const uint8_t& leftTwo, const uint8_t& rightOne, uint8_t rightTwo)
    : chip(chip), leftOne(leftOne), leftTwo(leftTwo), rightOne(rightOne), rightTwo(rightTwo)
{ 
    handle = lgGpiochipOpen(chip);
    if (handle < 0) {
        fprintf(stderr, "Error opening GPIO chip %d\n", chip);
    }

    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftOne, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftTwo, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightOne, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightTwo, 1);
}

MotorController::~MotorController()
{
    setMotorSpeed(0.0, 0.0);
    usleep(10000);
    lgGpiochipClose(handle);
}

bool MotorController::setMotorSpeed(const float left, const float right)
{
    float leftPWM = speedToPWM(left);
    float rightPWM = speedToPWM(right);
    if (left > 0) 
    {
        lgTxPwm(handle, leftOne, 100, 0, 0, 0);
        lgTxPwm(handle, leftTwo, 100, leftPWM, 0, 0);

    }
    else if (left < 0)
    {
        lgTxPwm(handle, leftOne, 100, leftPWM, 0, 0);
        lgTxPwm(handle, leftTwo, 100, 0, 0, 0);

    }
    else 
    {
        lgTxPwm(handle, leftOne, 100, 0, 0, 0);
        lgTxPwm(handle, leftTwo, 100, 0, 0, 0);
    }

    if (right > 0)
    {
        lgTxPwm(handle, rightOne, 100, 0, 0, 0);
        lgTxPwm(handle, rightTwo, 100, rightPWM, 0, 0);
    }
    else if (right < 0)
    {
        lgTxPwm(handle, rightOne, 100, rightPWM, 0, 0);
        lgTxPwm(handle, rightTwo, 100, 0, 0, 0);
    }
    else 
    {
        lgTxPwm(handle, rightOne, 100, 0, 0, 0);
        lgTxPwm(handle, rightTwo, 100, 0, 0, 0);
    }
    
    return true;
}

float MotorController::speedToPWM(const float speed) 
{
    float lower = -0.99;
    float upper = 0.99;
    return std::abs(std::max(lower, std::min(speed, upper)) * 100.0);
}