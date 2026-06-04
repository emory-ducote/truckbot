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
                    const double Kp = 1.25,
                    const double Ki = 0.0,
                    const double Kd = 0.0,
                    const double deadband = 0.0);
        ~MotorDriver();
        void setMeasuredSpeed(double v) { measuredSpeed.store(v); }
        double getMeasuredSpeed() const { return measuredSpeed.load(); }
        void setTargetSpeed(double v) { targetSpeed.store(v); }
        double getTargetSpeed() const { return targetSpeed.load(); }
        void updateMotorSpeed(const bool usePID);
    private:
        double speedToPWM(const double speed);
        const uint8_t chip;
        const uint8_t pinOne;
        const uint8_t pinTwo;
        int handle;
        std::atomic<double> measuredSpeed;
        std::atomic<double> targetSpeed;
        double integralSpeed,
               prevErrorSpeed,
               Kp,
               Ki,
               Kd,
               deadband,
               previousCalculationStamp;
};

#endif