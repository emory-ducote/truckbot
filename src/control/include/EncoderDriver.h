#ifndef ENCODERDRIVER_H_
#define ENCODERDRIVER_H_

#include <cstdint>
#include <lgpio.h>
#include <unordered_map>
#include <atomic>
#include <chrono>

class EncoderDriver {
    public:
        EncoderDriver(const uint8_t& chip, 
                      const uint8_t& pinA,
                      const uint8_t& pinB,
                      const double& wheelRadius,
                      const int& encoderCPR,
                      const int& encoderTicksPerRevolution);
        ~EncoderDriver();
        void handleEdgeChange();
        float getWheelSpeeds();  
        
    private:
        const uint8_t chip;
        const uint8_t pinA;
        const uint8_t pinB;
        const double wheelRadius;
        const int encoderCPR;
        const int encoderTicksPerRevolution;
        std::atomic<int> encoderTicks;
        int lastEncoderTicks;
        int lastEncoderRead;
        double previousCalculationStamp;
        int handle;     
};

#endif