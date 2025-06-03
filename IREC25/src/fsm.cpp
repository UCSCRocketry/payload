#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <SimpleFOC.h>

// ========== PROGRAM SETTINGS ==========
const bool enableSerial = true;
bool enableLogging = true;
const int EJECT_DELAY = 500;
const float LAUNCH_THRESHOLD = 15.0;
const float APOGEE_THRESHOLD = -1.0;
const int LOG_DELAY = 100;

const float L_MOTOR_INIT_POS = 0.0;
const float R_MOTOR_INIT_POS = 0.0;

const float UNLOCK_FIN_POS = 10.0;

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
#define SD_CS_PIN BUILTIN_SDCARD

// ========== IMU ==========
struct IMUData
{
    float ax, ay, az;
    float gx, gy, gz;
};

IMUData data_imu;
constexpr uint8_t SLAVE_ADDR = 0x10;
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

// ========== FLAGS ==========
bool finsDeployed = false;
bool ejectTimerStarted = false;

// ========== MISC ==========
unsigned long ejectStartTime = 0;

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
            myFile.println("Time,State,AX,AY,AZ,GX,GY,GZ,Enc1,Enc2,M1,M2");
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

// ===== Request IMU Data =====
void requestIMUData()
{
    data_imu = IMUData();
    Wire.requestFrom(SLAVE_ADDR, sizeof(IMUData));
    Wire.readBytes(reinterpret_cast<char *>(&data_imu), sizeof(IMUData));
}

void setup()
{
    if (enableSerial) {
        Serial.begin(9600);
    }

    while (enableSerial && !Serial)
        ;

    delay(100);

    
    enc_l.init();
    enc_r.init();
    SPI.begin();
    Wire.begin();

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
    requestIMUData();
    static unsigned long lastLogTime = 0;

    switch (state)
    {
    case IDLE:
        if (data_imu.az > LAUNCH_THRESHOLD)
        {
            state = LAUNCH;
            log("State: LAUNCH");
        }
        break;

    case LAUNCH:
        if (previousAz > 0 && data_imu.az < APOGEE_THRESHOLD)
        {
            state = APOGEE;
            log("State: APOGEE");
        }
        break;

    case APOGEE:
        if (!finsDeployed)
        {
            if (!ejectTimerStarted)
            {
                zero_off_l = l_integrated;
                zero_off_r = r_integrated;

                m_l.target = zero_off_l + UNLOCK_FIN_POS * _PI / 180.0;
                m_r.target = zero_off_r + UNLOCK_FIN_POS * _PI / 180.0;
                m_l.move();
                m_r.move();

                ejectStartTime = millis();
                ejectTimerStarted = true;
            }

            if (millis() - ejectStartTime >= EJECT_DELAY)
            {
                finsDeployed = true;
                state = DESCENT;
                log("State: DESCENT");

                ejectTimerStarted = false;
            }
        }
        break;

    case DESCENT:
        float roll = data_imu.ay;
        float correction = -roll * 0.05;

        m_l.target = zero_off_l + correction;
        m_r.target = zero_off_r - correction;

        m_l.move();
        m_r.move();
        break;
    }

    previousAz = data_imu.az;

    if (millis() - lastLogTime > LOG_DELAY)
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
        myFile.print(data_imu.ax, 2);
        myFile.print(",");
        myFile.print(data_imu.ay, 2);
        myFile.print(",");
        myFile.print(data_imu.az, 2);
        myFile.print(",");
        myFile.print(data_imu.gx, 2);
        myFile.print(",");
        myFile.print(data_imu.gy, 2);
        myFile.print(",");
        myFile.print(data_imu.gz, 2);
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
