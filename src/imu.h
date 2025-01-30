#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "Arduino_BMI270_BMM150.h"  // Include the correct IMU library

// PID Controller class
class PIDController {
public:
    PIDController(float Kp, float Ki, float Kd);
    float compute(float setpoint, float actualValue);
    void reset();

private:
    float Kp, Ki, Kd;
    float previousError;
    float integral;
};

// IMUHandler class to manage the accelerometer data
class IMUHandler {
public:
    IMUHandler();
    void update();
    float getXAcceleration();
    float getYAcceleration();
    float getZAcceleration();
    float getXGyro();
    float getYGyro();
    float getZGyro();
    int gyroAvail();

private:
    float accX, accY, accZ, gyroX, gyroY, gyroZ;
};

#endif
