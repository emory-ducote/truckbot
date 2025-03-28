#ifndef MICROCONTROLLER_H_
#define MICROCONTROLLER_H_

#include <lgpio.h>

class MotorController {
    public:
        MotorController(const uint8_t& chip, const uint8_t& leftOne, const uint8_t& leftTwo, const uint8_t& rightOne, uint8_t rightTwo);
        ~MotorController();

        bool setMotorSpeed(const float left, const float right);
        
    private:
        const uint8_t chip;
        const uint8_t leftOne;
        const uint8_t leftTwo;
        const uint8_t rightOne;
        const uint8_t rightTwo;
        int handle;

        float speedToPWM(const float speed);
};

#endif