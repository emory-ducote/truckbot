#include <iostream>
#include <cmath>
#include "EncoderDriver.h"

void encoderCallback(int e, lgGpioAlert_p evt, void *data)
{
   EncoderDriver * encoder_driver = static_cast<EncoderDriver*>(data);
   encoder_driver->handleEdgeChange();
}

EncoderDriver::EncoderDriver(const uint8_t& chip, 
                             const uint8_t& pinA,
                             const uint8_t& pinB)
    : chip(chip), 
      pinA(pinA), 
      pinB(pinB),
      encoderCount(0),
      lastEncoded(0)
    { 
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

    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        encoderCount++;
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        encoderCount--;

    lastEncoded = encoded;
}

float EncoderDriver::getWheelSpeeds(float dt)
{
    float deltaCount = static_cast<float>(encoderCount - lastEncoderCount) / CPR_output;
    float omega  = (deltaCount  * 2.0 * M_PI) / dt; // rad/s
    float vel    = omega  * 0.03; // m/s
    lastEncoderCount = encoderCount;
    return vel;
}