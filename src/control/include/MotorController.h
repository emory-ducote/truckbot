#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <lgpio.h>

class MotorController {
    public:
        MotorController(const uint8_t& chip, 
                        const uint8_t& leftFrontOne,
                        const uint8_t& leftFrontTwo,
                        const uint8_t& leftRearOne,
                        const uint8_t& leftRearTwo,
                        const uint8_t& rightFrontOne,
                        const uint8_t& rightFrontTwo,
                        const uint8_t& rightRearOne,
                        const uint8_t& rightRearTwo,
                        const uint8_t& liftOne,
                        const uint8_t& liftTwo);
        ~MotorController();
        bool applySpeedCommand(double linearX, double angularZ);
        bool moveActuator(const bool direction);
        
    private:
        const uint8_t chip;
        const uint8_t leftFrontOne;
        const uint8_t leftFrontTwo;
        const uint8_t leftRearOne;
        const uint8_t leftRearTwo;
        const uint8_t rightFrontOne;
        const uint8_t rightFrontTwo;
        const uint8_t rightRearOne;
        const uint8_t rightRearTwo;
        const uint8_t liftOne;
        const uint8_t liftTwo;
        int handle;
        
        bool setMotorSpeed(const double left, const double right);
        double speedToPWM(const double speed);
};

#endif