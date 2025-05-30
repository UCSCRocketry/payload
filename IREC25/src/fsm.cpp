#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Arduino_LSM9DS1.h>
#include <SimpleFOC.h>

// ========== PROGRAM SETTINGS ==========
const bool enableSerial = true;
const int EJECT_DELAY = 500;
// ========== STATE DEFINITIONS ==========
enum State
{
    IDLE,
    LAUNCH,
    APOGEE,
    DESCENT
};
State state = IDLE;



// ========== PIN DEFINITIONS ==========
#define ENC_CS_PIN 10
#define SD_CS_PIN 4

// ========== IMU ==========
float ax, ay, az;
float gx, gy, gz;
float previousAz = 0;
float roll = 0.0;
unsigned long lastRollUpdate = 0;

// ========== ACCELERATION THRESHOLDS ==========
const float LAUNCH_THRESHOLD = 15.0;
const float APOGEE_THRESHOLD = -1.0;

// ========== MOTOR SETUP ==========
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver6PWM driver1 = BLDCDriver6PWM(5, 6, 9, 10, 3, 11);

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver6PWM driver2 = BLDCDriver6PWM(5, 6, 9, 10, 3, 11);

// ========== ENCODERS ==========
uint16_t readAngle();

// ========== SD LOGGING ==========
File myFile;
int fileIndex = 0;
char filename[20];
bool sdInitialized = false;
bool fileCreated = false;
bool enableLogging = true;

// ========== FLAGS ==========
bool finsDeployed = false;

// ===== Function: Print Debug Messages =====
void log(String msg)
{
    if(enableSerial) {
        Serial.println(msg);
    }
}

// ===== Count Existing Log Files =====
void countFiles()
{
    fileIndex = 0;
    File root = SD.open("/");
    while (File entry = root.openNextFile())
    {
        fileIndex++;
        entry.close();
    }
    root.close();
}

// ===== Initialize SD and Create Unique File =====
void initSD()
{
    if (SD.begin(SD_CS_PIN))
    {
        log("SD init!");
        sdInitialized = true;
        countFiles();
        sprintf(filename, "log_%d.txt", fileIndex);
        myFile = SD.open(filename, FILE_WRITE);
        if (myFile)
        {
            myFile.println("===DATA START===");
            myFile.println("Time,State,AX,AY,AZ,GX,GY,GZ,Enc1,Enc2,M1PWM,M2PWM");
            myFile.close();
            fileCreated = true;
            log("Writing to " + String(filename));
        }
        else
        {
            log("Failed to create file.");
        }
    }
    else
    {
        log("SD init failed.");
    }
}

void setup()
{
    Serial.begin(9600);
    while (enableSerial && !Serial)
        ;

    delay(100);

    if (!IMU.begin())
    {
        Serial.println("Failed to initialize IMU!");
        while (1)
            ;
    }

    SPI.begin();
    pinMode(ENC_CS_PIN, OUTPUT);
    digitalWrite(ENC_CS_PIN, HIGH);

    initSD();

    driver1.voltage_power_supply = 5.0;
    driver1.init();
    motor1.linkDriver(&driver1);
    motor1.init();
    motor1.initFOC();

    driver2.voltage_power_supply = 5.0;
    driver2.init();
    motor2.linkDriver(&driver2);
    motor2.init();
    motor2.initFOC();

    motor1.move(0);
    motor2.move(0);
}

void loop()
{
    static unsigned long lastLogTime = 0;

    if (IMU.accelerationAvailable())
    {
        IMU.readAcceleration(ax, ay, az);
    }

    if (IMU.gyroscopeAvailable())
    {
        IMU.readGyroscope(gx, gy, gz);
    }

    switch (state)
    {
    case IDLE:
        if (az > LAUNCH_THRESHOLD)
        {
            state = LAUNCH;
            Serial.println("State: LAUNCH");
        }
        break;

    case LAUNCH:
        if (previousAz > 0 && az < APOGEE_THRESHOLD)
        {
            state = APOGEE;
            Serial.println("State: APOGEE");
        }
        break;

    case APOGEE:
        if (!finsDeployed)
        {
            motor1.target = 10.0 * _PI / 180.0;
            motor2.target = 10.0 * _PI / 180.0;
            motor1.move();
            motor2.move();
            delay(EJECT_DELAY);
            finsDeployed = true;
            state = DESCENT;
            Serial.println("State: DESCENT");
        }
        break;

    case DESCENT:
        float roll = getRollFromIMU();
        float correction = -roll * 0.05;
        motor1.move(correction);
        motor2.move(-correction);
        break;
    }

    previousAz = az;

    if (millis() - lastLogTime > 100)
    {
        lastLogTime = millis();
        logData();
    }

    motor1.loopFOC();
    motor2.loopFOC();
}

// ===== Rotary Encoder Read Function =====
uint16_t readAngle()
{
    digitalWrite(ENC_CS_PIN, LOW);
    SPI.transfer16(0xFFFF);
    digitalWrite(ENC_CS_PIN, HIGH);

    delayMicroseconds(1);

    digitalWrite(ENC_CS_PIN, LOW);
    uint16_t result = SPI.transfer16(0xFFFF);
    digitalWrite(ENC_CS_PIN, HIGH);

    return result & 0x3FFF;
}

// ===== Logging Function =====
void logData()
{
    if (!sdInitialized || !fileCreated)
        return;

    uint16_t enc1 = readAngle();
    uint16_t enc2 = readAngle();

    myFile = SD.open(filename, FILE_WRITE);
    if (myFile)
    {
        myFile.print(millis());
        myFile.print(",");
        myFile.print(state);
        myFile.print(",");
        myFile.print(ax, 2);
        myFile.print(",");
        myFile.print(ay, 2);
        myFile.print(",");
        myFile.print(az, 2);
        myFile.print(",");
        myFile.print(gx, 2);
        myFile.print(",");
        myFile.print(gy, 2);
        myFile.print(",");
        myFile.print(gz, 2);
        myFile.print(",");
        myFile.print(enc1);
        myFile.print(",");
        myFile.print(enc2);
        myFile.print(",");
        myFile.print(motor1.target);
        myFile.print(",");
        myFile.println(motor2.target);
        myFile.close();
    }
}

// ===== Calculate Roll from IMU =====
float getRollFromIMU()
{
    static const float alpha = 0.98;
    unsigned long now = millis();
    float dt = (now - lastRollUpdate) / 1000.0;
    if (dt <= 0.0 || dt > 1.0)
        dt = 0.01; // clamp if first call or jump
    lastRollUpdate = now;

    float axTemp, ayTemp, azTemp;
    float gxTemp, gyTemp, gzTemp;

    if (IMU.accelerationAvailable())
        IMU.readAcceleration(axTemp, ayTemp, azTemp);
    if (IMU.gyroscopeAvailable())
        IMU.readGyroscope(gxTemp, gyTemp, gzTemp);

    // Gyroscope gxTemp is in degrees/sec — convert to rad/sec
    float gyroRollRate = gxTemp * PI / 180.0;

    // Accelerometer roll (absolute) in radians
    float accelRoll = atan2(ayTemp, azTemp);

    // Complementary filter
    roll = alpha * (roll + gyroRollRate * dt) + (1.0 - alpha) * accelRoll;

    return roll;
}