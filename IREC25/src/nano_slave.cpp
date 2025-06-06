#include <Wire.h>
#include <SPI.h>
#include "Arduino_BMI270_BMM150.h"

const byte SLAVE_ADDR = 0x08;
char imuString[64];

// === Encoder CS Pins ===
const int ENC1_CS = 10; // PINK SIDE
const int ENC2_CS = 8;  // RED SIDE

SPIClass &SPI_ENCODERS = SPI;

// === Setup Encoders ===
void setupEncoders()
{
    pinMode(ENC1_CS, OUTPUT);
    pinMode(ENC2_CS, OUTPUT);
    digitalWrite(ENC1_CS, HIGH);
    digitalWrite(ENC2_CS, HIGH);

    SPI_ENCODERS.begin();
}

// === Read AS5047D via SPI ===
uint16_t readAS5047D(int csPin)
{
    uint16_t command = 0x3FFF; // Read angle command
    uint16_t nop = 0xFFFF;

    SPI_ENCODERS.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

    // First read: send 0x3FFF to request angle
    digitalWrite(csPin, LOW);
    delayMicroseconds(1);
    SPI_ENCODERS.transfer16(command);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(1);

    // Second read: retrieve the result of the last command
    digitalWrite(csPin, LOW);
    delayMicroseconds(1);
    uint16_t result = SPI_ENCODERS.transfer16(nop);
    digitalWrite(csPin, HIGH);

    SPI_ENCODERS.endTransaction();

    return result & 0x3FFF; // Return 14-bit angle
}

// === I2C Send Handler ===
void sendIMUString()
{
    Wire.write(imuString);
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(SLAVE_ADDR);
    Wire.onRequest(sendIMUString);

    setupEncoders();

    if (!IMU.begin())
    {
        Serial.println("We fucked");
        while (1);
    }
}

void loop()
{
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;

    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
    }
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
    }

    uint16_t angle1 = readAS5047D(ENC1_CS);
    uint16_t angle2 = readAS5047D(ENC2_CS);

    snprintf(imuString, sizeof(imuString),
             "ax=%.1f,ay=%.1f,az=%.1f,gx=%.1f,gy=%.1f,gz=%.1f,e1=%u,e2=%u",
             ax, ay, az, gx, gy, gz, angle1, angle2);

    delay(50);
}
