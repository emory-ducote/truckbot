#include <iostream>
#include <cmath>
#include <stdexcept>
#include <string>
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
      readErrors(0),
      lastEncoderTicks(0),
      lastEncoderRead(0),
      lastDtPrintStamp(0.0)
{ 
    previousCalculationStamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();    

    handle = lgGpiochipOpen(chip);
    if (handle < 0) {
        throw std::runtime_error("Error opening GPIO chip " + std::to_string(chip) +
                                 " for encoder (lgpio error " + std::to_string(handle) + ")");
    }

    checkGpio(lgGpioClaimInput(handle, LG_SET_PULL_NONE, pinA), "claim input", pinA);
    checkGpio(lgGpioClaimInput(handle, LG_SET_PULL_NONE, pinB), "claim input", pinB);

    // Register callbacks
    checkGpio(lgGpioSetAlertsFunc(handle, pinA, encoderCallback, this), "set alert callback", pinA);
    checkGpio(lgGpioSetAlertsFunc(handle, pinB, encoderCallback, this), "set alert callback", pinB);

    checkGpio(lgGpioClaimAlert(handle, 0, LG_BOTH_EDGES, pinA, -1), "claim alert", pinA);
    checkGpio(lgGpioClaimAlert(handle, 0, LG_BOTH_EDGES, pinB, -1), "claim alert", pinB);
}

void EncoderDriver::checkGpio(int result, const char* action, uint8_t pin)
{
    if (result < 0) {
        lgGpiochipClose(handle);
        throw std::runtime_error("Encoder GPIO " + std::string(action) + " failed on pin " +
                                 std::to_string(pin) + " (lgpio error " + std::to_string(result) + ")");
    }
}

EncoderDriver::~EncoderDriver()
{
}

void EncoderDriver::handleEdgeChange()
{
    int MSB = lgGpioRead(handle, pinA);
    int LSB = lgGpioRead(handle, pinB);
    if (MSB < 0 || LSB < 0) {
        readErrors++;
        return;
    }

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
    // Called at the odom publish rate, so only print dt once a second.
    if (curTime - lastDtPrintStamp >= 1.0)
    {
        std::cout << "DT: " << dt << std::endl;
        lastDtPrintStamp = curTime;
    }
    float rotations = static_cast<float>(encoderTicks - lastEncoderTicks) 
                                      / (encoderCPR * encoderTicksPerRevolution);
    float omega  = (rotations * 2.0 * M_PI) / dt; // rad/s
    float vel = omega  * wheelRadius; // m/s
    lastEncoderTicks = encoderTicks;
    previousCalculationStamp = curTime;
    return vel;
}