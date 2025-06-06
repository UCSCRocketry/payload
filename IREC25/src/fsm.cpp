#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// ========== PROGRAM SETTINGS ==========
const bool enableSerial = true;
bool enableLogging = true;
const int EJECT_DELAY = 500;
const float LAUNCH_THRESHOLD = 15.0;
const float APOGEE_THRESHOLD = -1.0;
const int LOG_DELAY = 100;
const int IMU_DELAY = 50;

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
struct __attribute__((packed)) IMUData
{
    float ax, ay, az;
    float gx, gy, gz;
};

IMUData data_imu;
constexpr uint8_t SLAVE_ADDR = 0x08;
float previousAz = 0;
float roll = 0.0;
unsigned long lastRollUpdate = 0;

char imuBuffer[100];

// ========== MOTOR SETUP ==========

// ========== ENCODERS ==========


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

float parseFloatValue(const char *str, const char *key)
{
    const char *found = strstr(str, key);
    if (!found)
        return 0.0f;
    return atof(found + strlen(key));
}

String formatIMUData(const IMUData &d)
{
    return "Accel: ax=" + String(d.ax, 3) +
           " ay=" + String(d.ay, 3) +
           " az=" + String(d.az, 3) +
           " | Gyro: gx=" + String(d.gx, 3) +
           " gy=" + String(d.gy, 3) +
           " gz=" + String(d.gz, 3);
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


// ===== Request IMU Data =====
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

    // Parse floats from the string buffer
    data_imu.ax = parseFloatValue(imuBuffer, "ax=");
    data_imu.ay = parseFloatValue(imuBuffer, "ay=");
    data_imu.az = parseFloatValue(imuBuffer, "az=");
    data_imu.gx = parseFloatValue(imuBuffer, "gx=");
    data_imu.gy = parseFloatValue(imuBuffer, "gy=");
    data_imu.gz = parseFloatValue(imuBuffer, "gz=");

}

void logData();

void setup()
{
    if (enableSerial) {
        Serial.begin(9600);
        while(!Serial);
    }   
    Wire.begin();
    delay(100);
   
}

void loop()
{
    static unsigned long lastLogTime = 0;
    static unsigned long lastIMURequestTime = 0;

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

            break;
    }

    previousAz = data_imu.az;

    uint32_t now = millis();

    if (now - lastIMURequestTime > IMU_DELAY)
    {
        requestIMUData();
        lastIMURequestTime = now; // Update IMU request timer
    }

    if (now - lastLogTime > LOG_DELAY)
    {
        lastLogTime = now; // Update logging timer
        logData();
    }
}

// ===== Logging Function =====
void logData()
{
    log(formatIMUData(data_imu));

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
        // myFile.print(current_l_angle);
        myFile.print(",");
        // myFile.print(current_r_angle);
        myFile.print(",");
        // myFile.print(m_l.target);
        myFile.print(",");
        // myFile.println(m_r.target);
        myFile.close();
    }
    
}