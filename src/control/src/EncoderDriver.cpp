#include <iostream>
#include <cmath>
#include "EncoderDriver.h"

void encoderCallback(int /*e*/, lgGpioAlert_p /*evt*/, void *data)
{
   EncoderDriver * encoder_driver = static_cast<EncoderDriver*>(data);
   encoder_driver->handleEdgeChange();
}

EncoderDriver::EncoderDriver(const uint8_t& chip, 
                             const uint8_t& pinA,
                             const uint8_t& pinB,
                             const double& wheelRadius,
                             const int& encoderCPR,
                             const int& encoderTicksPerRevolution)
    : chip(chip), 
      pinA(pinA), 
      pinB(pinB),
      wheelRadius(wheelRadius),
      encoderCPR(encoderCPR),
      encoderTicksPerRevolution(encoderTicksPerRevolution),
      encoderTicks(0),
      lastEncoderRead(0)
{ 
    previousCalculationStamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();    

    handle = lgGpiochipOpen(chip);
    if (handle < 0) {
        fprintf(stderr, "Error opening GPIO chip %d\n", chip);
    }

    lgGpioClaimInput(handle, LG_SET_PULL_NONE, pinA);
    lgGpioClaimInput(handle, LG_SET_PULL_NONE, pinB);

    // Register callbacks
    lgGpioSetAlertsFunc(handle, pinA, encoderCallback, this);
    lgGpioSetAlertsFunc(handle, pinB, encoderCallback, this);

    lgGpioClaimAlert(handle, 0, LG_BOTH_EDGES, pinA, -1);
    lgGpioClaimAlert(handle, 0, LG_BOTH_EDGES, pinB, -1);
}

EncoderDriver::~EncoderDriver()
{
}

void EncoderDriver::handleEdgeChange()
{
    int MSB = lgGpioRead(handle, pinA);
    int LSB = lgGpioRead(handle, pinB);

    int currentEncoderRead = (MSB << 1) | LSB;
    int encoderEdgeChange = (lastEncoderRead << 2) | currentEncoderRead;

    if (encoderEdgeChange == 0b1101 || encoderEdgeChange == 0b0100 || encoderEdgeChange == 0b0010 || encoderEdgeChange == 0b1011)
        encoderTicks++;
    else if (encoderEdgeChange == 0b1110 || encoderEdgeChange == 0b0111 || encoderEdgeChange == 0b0001 || encoderEdgeChange == 0b1000)
        encoderTicks--;

    lastEncoderRead = currentEncoderRead;
}

double EncoderDriver::getWheelSpeeds()
{
    double curTime = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
    double dt = curTime - previousCalculationStamp;
    std::cout << "DT: " << dt << std::endl;
    float rotations = static_cast<float>(encoderTicks - lastEncoderTicks) 
                                      / (encoderCPR * encoderTicksPerRevolution);
    float omega  = (rotations * 2.0 * M_PI) / dt; // rad/s
    float vel = omega  * wheelRadius; // m/s
    lastEncoderTicks = encoderTicks;
    previousCalculationStamp = curTime;
    return vel;
}