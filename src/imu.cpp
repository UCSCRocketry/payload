#include "imu.h"
#include "Arduino_BMI270_BMM150.h"

// PIDController implementation
PIDController::PIDController(float Kp, float Ki, float Kd)
    : Kp(Kp), Ki(Ki), Kd(Kd), previousError(0.0), integral(0.0) {}

float PIDController::compute(float setpoint, float actualValue) {
    float error = setpoint - actualValue;
    integral += error;
    float derivative = error - previousError;

    float output = Kp * error + Ki * integral + Kd * derivative;

    previousError = error;

    return output;  
}

void PIDController::reset() {
    previousError = 0.0;
    integral = 0.0;
}

// IMUHandler implementation
IMUHandler::IMUHandler() : accX(0), accY(0), accZ(0), gyroX(0), gyroY(0), gyroZ(0) {}

void IMUHandler::update() {
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accX, accY, accZ);
    }

    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
    }
}

// Getter methods for acceleration
float IMUHandler::getXAcceleration() { return accX; }
float IMUHandler::getYAcceleration() { return accY; }
float IMUHandler::getZAcceleration() { return accZ; }

// Getter methods for gyroscope
float IMUHandler::getXGyro() { return gyroX; } // Angular velocity in °/s
float IMUHandler::getYGyro() { return gyroY; }
float IMUHandler::getZGyro() { return gyroZ; }
