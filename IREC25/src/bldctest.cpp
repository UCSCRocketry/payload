#include <SimpleFOC.h>
#include <Arduino.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(5, 6, 9, 10, 3, 11);

// Default velocity magnitude (can be changed later)
float velocity_magnitude = 10.0;

// Movement control variables
bool moving = false;
unsigned long move_start_time = 0;
float move_duration = 0;
float move_velocity = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for serial monitor (optional)

    // Driver config
    driver.voltage_power_supply = 5;
    driver.voltage_limit = 5;
    driver.pwm_frequency = 32000;
    driver.init();

    // Motor config
    motor.linkDriver(&driver);
    motor.voltage_limit = 5; // safety
    motor.controller = MotionControlType::velocity_openloop;
    motor.init();

    Serial.println("Motor ready!"); 
    Serial.println("Send command:");
    Serial.println("  R x  - Move right for x seconds");
    Serial.println("  L x  - Move left for x seconds");
    Serial.println("  V x  - Set velocity magnitude to x rad/s");
}

void loop()
{
    // Check serial input
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        char cmd = input.charAt(0);
        String valStr = input.substring(2);
        float val = valStr.toFloat();

        if ((cmd == 'R' || cmd == 'L') && val > 0)
        {
            move_duration = val * 1000.0; // convert to ms
            move_start_time = millis();
            move_velocity = (cmd == 'R') ? velocity_magnitude : -velocity_magnitude;
            moving = true;

            Serial.print("Moving ");
            Serial.print((cmd == 'R') ? "right" : "left");
            Serial.print(" for ");
            Serial.print(val);
            Serial.println(" seconds...");
        }

        else if (cmd == 'V' && val > 0)
        {
            velocity_magnitude = val;
            Serial.print("Velocity magnitude set to ");
            Serial.print(val);
            Serial.println(" rad/s");
        }

        else
        {
            Serial.println("Invalid command.");
        }
    }

    // Handle motor motion
    if (moving)
    {
        if (millis() - move_start_time < move_duration)
        {
            motor.move(move_velocity);
        }
        else
        {
            motor.move(0); // Stop
            moving = false;
            Serial.println("Motion complete.");
        }
    }
    else
    {
        motor.move(0); // Ensure motor is stopped
    }
}
