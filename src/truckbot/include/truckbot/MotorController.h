#ifndef MICROCONTROLLER_H_
#define MICROCONTROLLER_H_

#include <lgpio.h>

class MotorController {
    public:
        MotorController(const uint8_t& chip, const uint8_t& leftOne, const uint8_t& leftTwo, const uint8_t& rightOne, const uint8_t& rightTwo, const uint8_t& liftOne, const uint8_t& liftTwo);
        ~MotorController();
        bool setMotorSpeed(const float left, const float right);
        bool moveActuator(const bool direction);
        
    private:
        const uint8_t chip;
        const uint8_t leftOne;
        const uint8_t leftTwo;
        const uint8_t rightOne;
        const uint8_t rightTwo;
        const uint8_t liftOne;
        const uint8_t liftTwo;
        int handle;

        float speedToPWM(const float speed);
};

#endif