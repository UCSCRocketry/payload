#include <Arduino.h>
#include <math.h>
#include <SPI.h>

float target_position1 = 0.0f; // radians
float target_position2 = 0.0f;

const float Kp_position = 5.0f; // tune this experimentally


// === Motor 1 Phase Pins ===
const int AH1 = 2, AL1 = 3;
const int BH1 = 4, BL1 = 5;
const int CH1 = 6, CL1 = 9;

// === Motor 2 Phase Pins ===
const int AH2 = 7, AL2 = 8;
const int BH2 = 10, BL2 = 12;
const int CH2 = 14, CL2 = 15;

// === Encoder CS Pins ===
const int ENC1_CS = 10;
const int ENC2_CS = 10;

// === Encoder Setup ===
SPIClass &SPI_ENCODERS = SPI1;
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

// == Setup ENC ===
void setupEncoders()
{
    pinMode(ENC1_CS, OUTPUT);
    pinMode(ENC2_CS, OUTPUT);
    digitalWrite(ENC1_CS, HIGH);
    digitalWrite(ENC2_CS, HIGH);

    SPI_ENCODERS.begin();
}

// === Setup PWM ===
void setupPWM(int pin)
{
    pinMode(pin, OUTPUT);
    analogWriteFrequency(pin, pwmFreq);
    analogWriteResolution(pwmRes);
}

// === ENC Read ==
uint16_t readAS5047D(int csPin)
{
    uint16_t command = 0x3FFF; // Read angle command (16-bit, parity checked optional)

    digitalWrite(csPin, LOW);
    delayMicroseconds(1);
    SPI_ENCODERS.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // up to 10 MHz, MODE1

    SPI_ENCODERS.transfer16(command); // throwaway read (AS5047D requires two reads)

    digitalWrite(csPin, HIGH);
    delayMicroseconds(1);

    digitalWrite(csPin, LOW);
    delayMicroseconds(1);
    uint16_t result = SPI_ENCODERS.transfer16(0x0000); // actual angle read
    digitalWrite(csPin, HIGH);

    SPI_ENCODERS.endTransaction();

    return result & 0x3FFF; // 14-bit angle
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
    while (!Serial)
        ;

    int pins[] = {AH1, AL1, BH1, BL1, CH1, CL1, AH2, AL2, BH2, BL2, CH2, CL2};
    for (int i = 0; i < 12; i++)
        setupPWM(pins[i]);

    setupEncoders();

    lastSwitch = millis();
    Serial.println("SVPWM Dual Motor Oscillator Ready!");
}

void loop()
{
    unsigned long now = micros();

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
        uint16_t angle_raw1 = readAS5047D(ENC1_CS);
        uint16_t angle_raw2 = readAS5047D(ENC2_CS);

        // === Track wraparounds ===
        int16_t delta1 = angle_raw1 - prev_raw1;
        if (delta1 > 8192)
            revolutions1--; // Wrapped backwards
        else if (delta1 < -8192)
            revolutions1++; // Wrapped forward

        int16_t delta2 = angle_raw2 - prev_raw2;
        if (delta2 > 8192)
            revolutions2--;
        else if (delta2 < -8192)
            revolutions2++;

        // === Compute full mechanical angles in radians ===
        full_angle1 = ((float)revolutions1 + (float)angle_raw1 / 16384.0f) * TWO_PI;
        full_angle2 = ((float)revolutions2 + (float)angle_raw2 / 16384.0f) * TWO_PI;

        // Save previous readings
        prev_raw1 = angle_raw1;
        prev_raw2 = angle_raw2;

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
