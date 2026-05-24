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

    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftFrontOne, 0);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftFrontTwo, 0);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftRearOne, 0);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, leftRearTwo, 0);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightFrontOne, 0);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightFrontTwo, 0);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightRearOne, 0);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, rightRearTwo, 0);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, liftOne, 0);
    lgGpioClaimOutput(handle, LG_SET_PULL_NONE, liftTwo, 0);

}

MotorController::~MotorController()
{
    setMotorSpeed(0.0, 0.0);
    usleep(10000);
    lgGpiochipClose(handle);
}

bool MotorController::applySpeedCommand(double linearX, double angularZ)
{
    // Angular scaling factor - tune this to adjust turn responsiveness
    double angularScaleFactor = 3;
    
    // Wheelbase in meters (8 inches ≈ 0.2032 m)
    const double wheelbase = 0.2032;
    
    // Account for different arc paths during turns
    // Front wheels travel a longer arc, rear wheels a shorter arc
    // Scale based on wheelbase and angular velocity
    double frontScale = 1.0 + (wheelbase / vehicleWidth) * std::abs(angularZ) * 0.5;
    double rearScale = 1.0 - (wheelbase / vehicleWidth) * std::abs(angularZ) * 0.5;

    // Compute individual wheel velocities for skid steer with wheelbase compensation
    double frontLeftVel = linearX - (vehicleWidth/2.0) * angularZ * angularScaleFactor * frontScale;
    double frontRightVel = linearX + (vehicleWidth/2.0) * angularZ * angularScaleFactor * frontScale;
    double rearLeftVel = linearX - (vehicleWidth/2.0) * angularZ * angularScaleFactor * rearScale;
    double rearRightVel = linearX + (vehicleWidth/2.0) * angularZ * angularScaleFactor * rearScale;

    // Optional minimum wheel command to overcome motor deadband
    const double minWheelSpeed = 0.15;

    if (std::abs(frontLeftVel) > 0.01)
        frontLeftVel = std::copysign(
            std::max(std::abs(frontLeftVel), minWheelSpeed),
            frontLeftVel);

    if (std::abs(frontRightVel) > 0.01)
        frontRightVel = std::copysign(
            std::max(std::abs(frontRightVel), minWheelSpeed),
            frontRightVel);

    if (std::abs(rearLeftVel) > 0.01)
        rearLeftVel = std::copysign(
            std::max(std::abs(rearLeftVel), minWheelSpeed),
            rearLeftVel);

    if (std::abs(rearRightVel) > 0.01)
        rearRightVel = std::copysign(
            std::max(std::abs(rearRightVel), minWheelSpeed),
            rearRightVel);

    // Convert m/s → RPM
    frontLeftVel /= wheelRadius;
    frontRightVel /= wheelRadius;
    rearLeftVel /= wheelRadius;
    rearRightVel /= wheelRadius;

    frontLeftVel *= (60.0/(2.0*M_PI));
    frontRightVel *= (60.0/(2.0*M_PI));
    rearLeftVel *= (60.0/(2.0*M_PI));
    rearRightVel *= (60.0/(2.0*M_PI));

    double scaleFactor =
        std::min(1.0,
            maxWheelMotorRpm /
            std::max({std::abs(frontLeftVel),
                      std::abs(frontRightVel),
                      std::abs(rearLeftVel),
                      std::abs(rearRightVel)}));

    frontLeftVel *= scaleFactor;
    frontRightVel *= scaleFactor;
    rearLeftVel *= scaleFactor;
    rearRightVel *= scaleFactor;

    setMotorSpeeds(
        frontLeftVel/maxWheelMotorRpm,
        frontRightVel/maxWheelMotorRpm,
        rearLeftVel/maxWheelMotorRpm,
        rearRightVel/maxWheelMotorRpm);

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

bool MotorController::setMotorSpeeds(const double frontLeft, const double frontRight, 
                                      const double rearLeft, const double rearRight)
{
    double flPWM = speedToPWM(frontLeft);
    double frPWM = speedToPWM(frontRight);
    double rlPWM = speedToPWM(rearLeft);
    double rrPWM = speedToPWM(rearRight);

    // Front left
    if (frontLeft > 0) 
    {
        lgTxPwm(handle, leftFrontOne, 100, 0, 0, 0);
        lgTxPwm(handle, leftFrontTwo, 100, flPWM, 0, 0);
    }
    else if (frontLeft < 0)
    {
        lgTxPwm(handle, leftFrontOne, 100, flPWM, 0, 0);
        lgTxPwm(handle, leftFrontTwo, 100, 0, 0, 0);
    }
    else 
    {
        lgTxPwm(handle, leftFrontOne, 100, 0, 0, 0);
        lgTxPwm(handle, leftFrontTwo, 100, 0, 0, 0);
    }

    // Front right
    if (frontRight > 0)
    {
        lgTxPwm(handle, rightFrontOne, 100, 0, 0, 0);
        lgTxPwm(handle, rightFrontTwo, 100, frPWM, 0, 0);
    }
    else if (frontRight < 0)
    {
        lgTxPwm(handle, rightFrontOne, 100, frPWM, 0, 0);
        lgTxPwm(handle, rightFrontTwo, 100, 0, 0, 0);
    }
    else 
    {
        lgTxPwm(handle, rightFrontOne, 100, 0, 0, 0);
        lgTxPwm(handle, rightFrontTwo, 100, 0, 0, 0);
    }

    // Rear left
    if (rearLeft > 0) 
    {
        lgTxPwm(handle, leftRearOne, 100, 0, 0, 0);
        lgTxPwm(handle, leftRearTwo, 100, rlPWM, 0, 0);
    }
    else if (rearLeft < 0)
    {
        lgTxPwm(handle, leftRearOne, 100, rlPWM, 0, 0);
        lgTxPwm(handle, leftRearTwo, 100, 0, 0, 0);
    }
    else 
    {
        lgTxPwm(handle, leftRearOne, 100, 0, 0, 0);
        lgTxPwm(handle, leftRearTwo, 100, 0, 0, 0);
    }

    // Rear right
    if (rearRight > 0)
    {
        lgTxPwm(handle, rightRearOne, 100, 0, 0, 0);
        lgTxPwm(handle, rightRearTwo, 100, rrPWM, 0, 0);
    }
    else if (rearRight < 0)
    {
        lgTxPwm(handle, rightRearOne, 100, rrPWM, 0, 0);
        lgTxPwm(handle, rightRearTwo, 100, 0, 0, 0);
    }
    else 
    {
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

