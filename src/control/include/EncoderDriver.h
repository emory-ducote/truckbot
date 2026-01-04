#ifndef ENCODERDRIVER_H_
#define ENCODERDRIVER_H_

#include <cstdint>
#include <lgpio.h>
#include <unordered_map>



class EncoderDriver {
    public:
        EncoderDriver(const uint8_t& chip, 
                      const uint8_t& pinA,
                      const uint8_t& pinB,
                      const double& wheelRadius,
                      const int& encoderCPR,
                      const int& encoderMultiplier);
        ~EncoderDriver();
        void handleEdgeChange();
        float getWheelSpeeds(float dt);  
        
    private:
        const uint8_t chip;
        const uint8_t pinA;
        const uint8_t pinB;
        const double wheelRadius;
        const int encoderCPR;
        const int encoderMultiplier;
        long encoderCount;
        long lastEncoderCount;
        int lastEncoded;
        int handle;     
};

#endif