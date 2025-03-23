#ifndef MICROCONTROLLER_H_
#define MICROCONTROLLER_H_

#include <lgpio.h>

class MotorController {
    public:
        MotorController(uint8_t chip, uint8_t leftOne, uint8_t leftTwo, uint8_t rightOne, uint8_t rightTwo);
        ~MotorController();

        bool setMotorSpeed(float left, float right);
        
    private:
        uint8_t chip;
        int handle;
        uint8_t leftOne;
        uint8_t leftTwo;
        uint8_t rightOne;
        uint8_t rightTwo;

        float speedToPWM(float speed);
};

#endif