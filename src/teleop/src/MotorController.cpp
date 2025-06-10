#include <cmath>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include "MotorController.h"

MotorController::MotorController(const uint8_t& chip, const uint8_t& leftOne, const uint8_t& leftTwo, const uint8_t& rightOne, const uint8_t& rightTwo, const uint8_t& liftOne, const uint8_t& liftTwo)
    : chip(chip), leftOne(leftOne), leftTwo(leftTwo), rightOne(rightOne), rightTwo(rightTwo), liftOne(liftOne), liftTwo(liftTwo)
{ 
    handle = lgGpiochipOpen(chip);
    if (handle < 0) {
        fprintf(stderr, "Error opening GPIO chip %d\n", chip);
    }

    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftOne, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftTwo, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightOne, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightTwo, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_DOWN, liftOne, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_DOWN, liftTwo, 1);

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
    // limit to 50 % to not saturate motors, also needs to be < 100%
    float lower = -0.99 / 2.0; 
    float upper = 0.99 / 2.0;
    
    return std::abs(std::max(lower, std::min(speed, upper)) * 100.0);
}

bool MotorController::moveActuator(const bool direction)
{
    std::cout << "direction: " << direction << std::endl;
    int command1 = 1;
    int command2 = 0;
    if (direction) {
        command1 = 0;
        command2 = 1;
    }
    lgGpioWrite(handle, liftOne, command1);
    lgGpioWrite(handle, liftTwo, command2);

    return true;
}
