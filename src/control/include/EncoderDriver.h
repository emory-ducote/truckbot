#ifndef ENCODERDRIVER_H_
#define ENCODERDRIVER_H_

#include <cstdint>
#include <lgpio.h>
#include <unordered_map>



class EncoderDriver {
    public:
        EncoderDriver(const uint8_t& chip, 
                        const uint8_t& pinA,
                        const uint8_t& pinB);
        ~EncoderDriver();
        void handleEdgeChange();
        double getRotations();  
        
    private:
        const uint8_t chip;
        const uint8_t pinA;
        const uint8_t pinB;
        long encoderCount;
        int lastEncoded;
        int handle;     
        const int CPR_output = 700 * 4;   // gearbox shaft resolution in X4 mode
};

#endif