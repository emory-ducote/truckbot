#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <lgpio.h>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

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
                        const uint8_t& liftTwo,
                        const double& vehicleWidth,
                        const double& wheelRadius, 
                        const int& maxWheelMotorRpm);
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
        const double vehicleWidth;
        const double wheelRadius;
        const int maxWheelMotorRpm;
        // Measured wheel speeds (m/s) provided by middleware
        std::atomic<double> measuredFL{0.0};
        std::atomic<double> measuredFR{0.0};
        std::atomic<double> measuredRL{0.0};
        std::atomic<double> measuredRR{0.0};

        // PID control thread and state
        std::atomic<double> targetFL{0.0};
        std::atomic<double> targetFR{0.0};
        std::atomic<double> targetRL{0.0};
        std::atomic<double> targetRR{0.0};
        std::thread controlThread;
        std::atomic<bool> controlRunning{false};
        std::mutex pidMutex;

        // PID gains (tunable)
        double Kp = 1.2;
        double Ki = 0.5;
        double Kd = 0.01;
        
        // setters/getters for measured values (called by middleware subscriptions)
    public:
        void setMeasuredFL(double v) { measuredFL.store(v); }
        void setMeasuredFR(double v) { measuredFR.store(v); }
        void setMeasuredRL(double v) { measuredRL.store(v); }
        void setMeasuredRR(double v) { measuredRR.store(v); }
        double getMeasuredFL() const { return measuredFL.load(); }
        double getMeasuredFR() const { return measuredFR.load(); }
        double getMeasuredRL() const { return measuredRL.load(); }
        double getMeasuredRR() const { return measuredRR.load(); }
        
        bool setMotorSpeed(const double left, const double right);
        bool setMotorSpeeds(const double frontLeft, const double frontRight, 
                           const double rearLeft, const double rearRight);
        double speedToPWM(const double speed);
};

#endif