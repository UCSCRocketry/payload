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
const float LAUNCH_THRESHOLD = 15.0;
const float APOGEE_THRESHOLD = -1.0;

const float L_MOTOR_INIT_POS = 0.0;
const float R_MOTOR_INIT_POS = 0.0;

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
#define ENC_L_CS_PIN 1
#define ENC_R_CS_PIN 2
#define SD_CS_PIN 4

// ========== IMU ==========
float ax, ay, az;
float gx, gy, gz;
float previousAz = 0;
float roll = 0.0;
unsigned long lastRollUpdate = 0;

// ========== MOTOR SETUP ==========
BLDCMotor m_l = BLDCMotor(7);
BLDCDriver6PWM driver_l = BLDCDriver6PWM(5, 6, 9, 10, 3, 11);

BLDCMotor m_r = BLDCMotor(7);
BLDCDriver6PWM driver_r = BLDCDriver6PWM(5, 6, 9, 10, 3, 11);

// ========== ENCODERS ==========
float zero_off_l = 0.0;
float l_prev_angle = L_MOTOR_INIT_POS;
float l_integrated = L_MOTOR_INIT_POS;
float current_l_angle = L_MOTOR_INIT_POS;
MagneticSensorSPI enc_l = MagneticSensorSPI(AS5047_SPI, ENC_L_CS_PIN);

float zero_off_r = 0.0;
float r_prev_angle = R_MOTOR_INIT_POS;
float r_integrated = R_MOTOR_INIT_POS;
float current_r_angle = R_MOTOR_INIT_POS;
MagneticSensorSPI enc_r = MagneticSensorSPI(AS5047_SPI, ENC_R_CS_PIN);


// ========== SD LOGGING ==========
File myFile;
int fileIndex = 0;
char filename[20];
bool sdInitialized = false;
bool fileCreated = false;
bool enableLogging = true;

// ========== FLAGS ==========
bool finsDeployed = false;
bool ejectTimerStarted = false;

// ========== MISC ==========
unsigned long ejectStartTime = 0;

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

void updateIntegratedAngles() {
    current_l_angle = enc_l.getAngle();
    current_r_angle = enc_r.getAngle();

    float delta_l = current_l_angle - l_prev_angle;
    if (delta_l > _PI)
        delta_l -= 2 * _PI;
    if (delta_l < -_PI)
        delta_l += 2 * _PI;
    l_integrated += delta_l;
    l_prev_angle = current_l_angle;

    float delta_r = current_r_angle - r_prev_angle;
    if (delta_r > _PI)
        delta_r -= 2 * _PI;
    if (delta_r < -_PI)
        delta_r += 2 * _PI;
    r_integrated += delta_r;
    r_prev_angle = current_r_angle;
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
    
    enc_l.init();
    enc_r.init();
    SPI.begin();

    initSD();

    driver_l.voltage_power_supply = 5.0;
    driver_l.init();
    m_l.linkDriver(&driver_l);
    m_l.linkSensor(&enc_l);
    m_l.init();
    m_l.initFOC();

    driver_r.voltage_power_supply = 5.0;
    driver_r.init();
    m_r.linkDriver(&driver_r);
    m_r.linkSensor(&enc_r);
    m_r.init();
    m_r.initFOC();

    m_l.target = L_MOTOR_INIT_POS;
    m_r.target = R_MOTOR_INIT_POS;

    m_l.move(0);
    m_r.move(0);
}

void loop()
{
    updateIntegratedAngles();
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
            log("State: LAUNCH");
        }
        break;

    case LAUNCH:
        if (previousAz > 0 && az < APOGEE_THRESHOLD)
        {
            state = APOGEE;
            log("State: APOGEE");
        }
        break;

    case APOGEE:
        if (!finsDeployed)
        {
            // Start the timer once
            if (!ejectTimerStarted)
            {
                // Store current zero offset and start timer
                zero_off_l = l_integrated;
                zero_off_r = r_integrated;

                // Move motors to 10 deg relative from current
                m_l.target = zero_off_l + 10.0 * _PI / 180.0;
                m_r.target = zero_off_r + 10.0 * _PI / 180.0;
                m_l.move();
                m_r.move();

                ejectStartTime = millis();
                ejectTimerStarted = true;
            }

            // Check if delay period passed without blocking
            if (millis() - ejectStartTime >= EJECT_DELAY)
            {
                finsDeployed = true;
                state = DESCENT;
                log("State: DESCENT");

                // Reset timer flag for future use (if needed)
                ejectTimerStarted = false;
            }
        }
        break;

    case DESCENT:
        float roll = getRollFromIMU();
        float correction = -roll * 0.05;

        // Adjust motors relative to their zero
        m_l.target = zero_off_l + correction;
        m_r.target = zero_off_r - correction;

        m_l.move();
        m_r.move();
        break;
    }

    previousAz = az;

    if (millis() - lastLogTime > 100)
    {
        lastLogTime = millis();
        logData();
    }

    m_l.loopFOC();
    m_r.loopFOC();
}

// ===== Logging Function =====
void logData()
{
    if (!sdInitialized || !fileCreated)
        return;

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
        myFile.print(current_l_angle);
        myFile.print(",");
        myFile.print(current_r_angle);
        myFile.print(",");
        myFile.print(m_l.target);
        myFile.print(",");
        myFile.println(m_r.target);
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