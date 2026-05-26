#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <lgpio.h>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

class MotorDriver {
    public:
        MotorDriver(const uint8_t& chip, 
                    const uint8_t& pinOne,
                    const uint8_t& pinTwo,
                    const double Kp,
                    const double Ki,
                    const double Kd);
        ~MotorDriver();
        void setMeasuredSpeed(double v) { measuredSpeed.store(v); }
        double getMeasuredSpeed() const { return measuredSpeed.load(); }
        void setMotorSpeed(const double targetSpeed, const bool usePID);
    private:
        double speedToPWM(const double speed);
        const uint8_t chip;
        const uint8_t pinOne;
        const uint8_t pinTwo;
        int handle;
        std::atomic<double> measuredSpeed;
        double integralSpeed,
               prevErrorSpeed,
               Kp,
               Ki,
               Kd,        
               previousCalculationStamp;
};

#endif