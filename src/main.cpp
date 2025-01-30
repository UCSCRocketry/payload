#include "imu.h"

PIDController pid(1.0, 0.1, 0.05);
IMUHandler imu;

// globals for gyro
float x, y, z;


void setup() {
    Serial.begin(9600);
    while (!Serial);

    Serial.println("Started");

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

}

void loop() {   
    imu.update();

    
    x = imu.getXGyro();
    y = imu.getYGyro();
    z = imu.getZGyro();

    Serial.print("Gyroscope (X, Y, Z): ");
    Serial.print(x, 3); Serial.print(", ");
    Serial.print(y, 3); Serial.print(", ");
    Serial.println(z, 3);
    
    float setpoint = 0.0;
    float actualValue = x;
    float controlSignal = pid.compute(setpoint, actualValue);

    Serial.print("Control Signal: ");
    Serial.println(controlSignal);

    delay(100);
}
