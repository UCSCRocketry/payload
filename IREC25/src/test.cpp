#include <Arduino.h>
#include <math.h>
#include <Wire.h>

float target_position1 = 0.0f; // radians
float target_position2 = 0.0f;

const float Kp_position = 5.0f; // tune this experimentally

constexpr uint8_t SLAVE_ADDR = 0x08;

// === Motor 1 Phase Pins ===
const int AH1 = 2, AL1 = 3;
const int BH1 = 4, BL1 = 5;
const int CH1 = 6, CL1 = 9;

// === Motor 2 Phase Pins ===
const int AH2 = 7, AL2 = 8;
const int BH2 = 10, BL2 = 12;
const int CH2 = 14, CL2 = 15;



// === Encoder Setup ===
int revolutions1 = 0, revolutions2 = 0;
uint16_t prev_raw1 = 0, prev_raw2 = 0;
float full_angle1 = 0.0f, full_angle2 = 0.0f;

// === PWM settings ===
const int pwmFreq = 20000;
const int pwmRes = 12;
const int pwmMax = (1 << pwmRes) - 1;

// === Motion parameters ===
float target_velocity = 75.0;    // rad/s
float voltage_amplitude = 0.9;   // [0..1], fraction of bus voltage
float ramp_rate = 300.0;         // rad/s²
unsigned long swing_time = 750; // ms

// === State variables ===
float angle1 = 0, angle2 = 0;
float current_velocity1 = 0, current_velocity2 = 0;
int direction = 1;
unsigned long lastControl = 0;
unsigned long lastSwitch = 0;
float dt = 0.0001f; // default 100 µs
uint16_t raw_1;
uint16_t raw_2;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;

const float ENC_TO_RAD = TWO_PI / 16384.0f;

char imuBuffer[100];

void requestIMUData()
{
    memset(imuBuffer, 0, sizeof(imuBuffer)); // Clear buffer

    Wire.requestFrom(SLAVE_ADDR, sizeof(imuBuffer));
    int i = 0;
    while (Wire.available() && i < sizeof(imuBuffer) - 1)
    {
        imuBuffer[i++] = Wire.read();
    }
    imuBuffer[i] = '\0'; // Null-terminate



    // Parse floats and integers from the string
    int parsed = sscanf(imuBuffer,
                        "ax=%f,ay=%f,az=%f,gx=%f,gy=%f,gz=%f,e1=%hu,e2=%hu",
                        &ax, &ay, &az, &gx, &gy, &gz, &raw_1, &raw_2);

    // Optional: Check if all 8 fields were successfully parsed
    if (parsed == 8)
    {
        // Do something with the parsed values
        Serial.print("AX: ");
        Serial.print(ax);
        Serial.print(" AY: ");
        Serial.print(ay);
        Serial.print(" AZ: ");
        Serial.print(az);
        Serial.print(" GX: ");
        Serial.print(gx);
        Serial.print(" GY: ");
        Serial.print(gy);
        Serial.print(" GZ: ");
        Serial.print(gz);
        Serial.print(" ENC1: ");
        Serial.print(raw_1);
        Serial.print(" ENC2: ");
        Serial.println(raw_2);
    }
    else
    {
        Serial.println("Failed to parse IMU string.");
    }
}

// === Setup PWM ===
void setupPWM(int pin)
{
    pinMode(pin, OUTPUT);
    analogWriteFrequency(pin, pwmFreq);
    analogWriteResolution(pwmRes);
}



// === SVPWM ===
void applySVPWM(int AH, int AL, int BH, int BL, int CH, int CL, float angle, float amplitude)
{
    float alpha = amplitude * cosf(angle);
    float beta = amplitude * sinf(angle);

    float Ua = alpha;
    float Ub = -0.5 * alpha + 0.8660254 * beta;
    float Uc = -0.5 * alpha - 0.8660254 * beta;

    float Umax = max(Ua, max(Ub, Uc));
    float Umin = min(Ua, min(Ub, Uc));
    float offset = 0.5 * (Umax + Umin);

    Ua -= offset;
    Ub -= offset;
    Uc -= offset;

    float dutyA = constrain(0.5 + 0.5 * Ua, 0.0, 1.0);
    float dutyB = constrain(0.5 + 0.5 * Ub, 0.0, 1.0);
    float dutyC = constrain(0.5 + 0.5 * Uc, 0.0, 1.0);

    analogWrite(AH, (int)(dutyA * pwmMax));
    analogWrite(AL, pwmMax - (int)(dutyA * pwmMax));
    analogWrite(BH, (int)(dutyB * pwmMax));
    analogWrite(BL, pwmMax - (int)(dutyB * pwmMax));
    analogWrite(CH, (int)(dutyC * pwmMax));
    analogWrite(CL, pwmMax - (int)(dutyC * pwmMax));
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    int pins[] = {AH1, AL1, BH1, BL1, CH1, CL1, AH2, AL2, BH2, BL2, CH2, CL2};
    for (int i = 0; i < 12; i++)
        setupPWM(pins[i]);


    lastSwitch = millis();
    Serial.println("SVPWM Dual Motor Oscillator Ready!");
    delay(100);
}

void loop()
{
    unsigned long now = micros();
    requestIMUData();

    if (millis() - lastSwitch >= swing_time)
    {
        direction *= -1;
        lastSwitch = millis();
        Serial.println(direction > 0 ? "Forward" : "Reverse");
    }

    if (now - lastControl >= 100)
    {
        dt = (now - lastControl) * 1e-6;
        lastControl = now;

        // Calculate position error
        float error1 = target_position1 - full_angle1;
        float error2 = target_position2 - full_angle2;

        // Simple proportional controller for velocity command
        float target_velocity1 = Kp_position * error1;
        float target_velocity2 = Kp_position * error2;

        // Optional: limit max velocity to prevent overshoot or too high speed
        const float max_velocity = 100.0f; // rad/s
        target_velocity1 = constrain(target_velocity1, -max_velocity, max_velocity);
        target_velocity2 = constrain(target_velocity2, -max_velocity, max_velocity);

        // Ramp velocity towards target velocity
        float max_step = ramp_rate * dt;

        float delta_vel1 = target_velocity1 - current_velocity1;
        float delta_vel2 = target_velocity2 - current_velocity2;

        current_velocity1 += constrain(delta_vel1, -max_step, max_step);
        current_velocity2 += constrain(delta_vel2, -max_step, max_step);

        
        
        // === Track wraparounds ===
        int16_t delta1 = raw_1 - prev_raw1;
        if (delta1 > 8192)
            revolutions1--; // Wrapped backwards
        else if (delta1 < -8192)
            revolutions1++; // Wrapped forward

        int16_t delta2 = raw_2 - prev_raw2;
        if (delta2 > 8192)
            revolutions2--;
        else if (delta2 < -8192)
            revolutions2++;


        // Save previous readings
        prev_raw1 = raw_1;
        prev_raw2 = raw_2;

        full_angle1 = (revolutions1 * 16384 + raw_1) * ENC_TO_RAD;
        full_angle2 = (revolutions2 * 16384 + raw_2) * ENC_TO_RAD;

        // === Compute electrical angle (assume pole pairs = 1, adjust if not) ===
        float electrical_angle1 = fmodf(full_angle1 * 7 + 0.05f * current_velocity1, TWO_PI);
        float electrical_angle2 = fmodf(full_angle2 * 7 + 0.05f * current_velocity2, TWO_PI);


        applySVPWM(AH1, AL1, BH1, BL1, CH1, CL1, electrical_angle1, voltage_amplitude);
        applySVPWM(AH2, AL2, BH2, BL2, CH2, CL2, electrical_angle2, voltage_amplitude);
    }

    if (Serial.available())
    {
        char cmd = Serial.read();
        float val = Serial.parseFloat();

        if (cmd == 'V')
        {
            target_velocity = constrain(val, 0.0, 100.0);
            Serial.print("Velocity: ");
            Serial.println(target_velocity);
        }
        else if (cmd == 'A')
        {
            voltage_amplitude = constrain(val, 0.0, 1.0);
            Serial.print("Amplitude: ");
            Serial.println(voltage_amplitude);
        }
        else if (cmd == 'T')
        {
            swing_time = constrain((int)val, 200, 10000);
            Serial.print("Swing Time: ");
            Serial.println(swing_time);
        }
        else if (cmd == 'P') // Set target position in degrees
        {
            float pos_deg = val;
            target_position1 = radians(pos_deg);
            target_position2 = radians(pos_deg);
            Serial.print("Target position set to ");
            Serial.println(pos_deg);
        }

        while (Serial.available())
            Serial.read();
    }
}
