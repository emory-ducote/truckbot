#include <cmath>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <algorithm>
#include "MotorController.h"

MotorController::MotorController(const uint8_t& chip, 
                                 const uint8_t& leftFrontOne,
                                 const uint8_t& leftFrontTwo,
                                 const uint8_t& leftRearOne,
                                 const uint8_t& leftRearTwo,
                                 const uint8_t& rightFrontOne,
                                 const uint8_t& rightFrontTwo,
                                 const uint8_t& rightRearOne,
                                 const uint8_t& rightRearTwo,
                                 const uint8_t& liftOne,
                                 const uint8_t& liftTwo,
                                 const double& vehicleWidth,
                                 const double& wheelRadius, 
                                 const int& maxWheelMotorRpm)
    : chip(chip), 
      leftFrontOne(leftFrontOne), 
      leftFrontTwo(leftFrontTwo), 
      leftRearOne(leftRearOne), 
      leftRearTwo(leftRearTwo), 
      rightFrontOne(rightFrontOne), 
      rightFrontTwo(rightFrontTwo),
      rightRearOne(rightRearOne),
      rightRearTwo(rightRearTwo),
      liftOne(liftOne),
      liftTwo(liftTwo),
      vehicleWidth(vehicleWidth),
      wheelRadius(wheelRadius),
      maxWheelMotorRpm(maxWheelMotorRpm)
{ 
    handle = lgGpiochipOpen(chip);
    if (handle < 0) {
        fprintf(stderr, "Error opening GPIO chip %d\n", chip);
    }

    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftFrontOne, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftFrontTwo, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftRearOne, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftRearTwo, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightFrontOne, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightFrontTwo, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightRearOne, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightRearTwo, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_DOWN, liftOne, 1);
    lgGpioClaimOutput(handle, LG_SET_PULL_DOWN, liftTwo, 1);

}

MotorController::~MotorController()
{
    setMotorSpeed(0.0, 0.0);
    usleep(10000);
    lgGpiochipClose(handle);
}

bool MotorController::applySpeedCommand(double linearX, double angularZ)
{
    double leftVel = linearX - (0.2 / 2) * angularZ;
    double rightVel = linearX + (0.2 / 2) * angularZ;
    leftVel = leftVel / wheelRadius;
    rightVel = rightVel / wheelRadius;
    leftVel = leftVel * (60 / (M_PI * 2));
    rightVel = rightVel * (60 / (M_PI * 2));
    double scaleFactor = std::min(1.0, maxWheelMotorRpm / std::max(std::abs(leftVel), std::abs(rightVel)));
    leftVel = (leftVel * scaleFactor) / maxWheelMotorRpm;
    rightVel = (rightVel * scaleFactor) / maxWheelMotorRpm;
    this->setMotorSpeed(leftVel, rightVel);
    return true;

}

bool MotorController::moveActuator(const bool direction)
{
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

bool MotorController::setMotorSpeed(const double left, const double right)
{
    double leftPWM = speedToPWM(left);
    double rightPWM = speedToPWM(right);
    if (left > 0) 
    {
        lgTxPwm(handle, leftFrontOne, 100, 0, 0, 0);
        lgTxPwm(handle, leftFrontTwo, 100, leftPWM, 0, 0);
        lgTxPwm(handle, leftRearOne, 100, 0, 0, 0);
        lgTxPwm(handle, leftRearTwo, 100, leftPWM, 0, 0);

    }
    else if (left < 0)
    {
        lgTxPwm(handle, leftFrontOne, 100, leftPWM, 0, 0);
        lgTxPwm(handle, leftFrontTwo, 100, 0, 0, 0);
        lgTxPwm(handle, leftRearOne, 100, leftPWM, 0, 0);
        lgTxPwm(handle, leftRearTwo, 100, 0, 0, 0);

    }
    else 
    {
        lgTxPwm(handle, leftFrontOne, 100, 0, 0, 0);
        lgTxPwm(handle, leftFrontTwo, 100, 0, 0, 0);
        lgTxPwm(handle, leftRearOne, 100, 0, 0, 0);
        lgTxPwm(handle, leftRearTwo, 100, 0, 0, 0);
    }

    if (right > 0)
    {
        lgTxPwm(handle, rightFrontOne, 100, 0, 0, 0);
        lgTxPwm(handle, rightFrontTwo, 100, rightPWM, 0, 0);
        lgTxPwm(handle, rightRearOne, 100, 0, 0, 0);
        lgTxPwm(handle, rightRearTwo, 100, rightPWM, 0, 0);
    }
    else if (right < 0)
    {
        lgTxPwm(handle, rightFrontOne, 100, rightPWM, 0, 0);
        lgTxPwm(handle, rightFrontTwo, 100, 0, 0, 0);
        lgTxPwm(handle, rightRearOne, 100, rightPWM, 0, 0);
        lgTxPwm(handle, rightRearTwo, 100, 0, 0, 0);
    }
    else 
    {
        lgTxPwm(handle, rightFrontOne, 100, 0, 0, 0);
        lgTxPwm(handle, rightFrontTwo, 100, 0, 0, 0);
        lgTxPwm(handle, rightRearOne, 100, 0, 0, 0);
        lgTxPwm(handle, rightRearTwo, 100, 0, 0, 0);
    }
    
    return true;
}

double MotorController::speedToPWM(const double speed) 
{
    // needs to be < 100%
    double lower = -0.99; 
    double upper = 0.99;
    
    return std::abs(std::max(lower, std::min(speed, upper)) * 100.0);
}

