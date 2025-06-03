#include <Arduino.h>

// List of PWM-capable pins on Teensy 4.1
const int pwmPins[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 12, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 36, 37};
const int numPins = sizeof(pwmPins) / sizeof(pwmPins[0]);

int currentPwmPin = -1;

bool isPwmCapable(int pin)
{
    for (int i = 0; i < numPins; i++)
    {
        if (pwmPins[i] == pin)
            return true;
    }
    return false;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for Serial Monitor
    Serial.println("Teensy PWM Pin Selector");
    Serial.println("Enter a PWM-capable pin number to start 50% duty PWM on that pin.");
    Serial.println("Example: type '3' and press Enter.");
}

void loop()
{
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0)
            return;

        int pin = input.toInt();
        if (!isPwmCapable(pin))
        {
            Serial.print("Pin ");
            Serial.print(pin);
            Serial.println(" is not PWM-capable.");
            return;
        }

        if (currentPwmPin != -1)
        {
            analogWrite(currentPwmPin, 0); // Turn off previous PWM
            Serial.print("Stopped PWM on pin ");
            Serial.println(currentPwmPin);
        }

        pinMode(pin, OUTPUT);
        analogWrite(pin, 128); // 50% duty cycle
        Serial.print("Started 50% PWM on pin ");
        Serial.println(pin);
        currentPwmPin = pin;
    }
}
