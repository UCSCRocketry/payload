#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Arduino_LSM9DS1.h>

// ===== Constants and Pins =====
const int S1_PIN = 5;
const int S2_PIN = 6;
const int SD_CS_PIN = 7;
const int ENC_CS_PIN = 10;

// ===== System Settings =====
const bool enableLogging = true;
bool enableSerial = true;

// ===== State Flags =====
bool imuAvailable = true;
bool apogeeDetected = false;
bool wasRising = false;
bool fileCreated = false;
bool sdInitialized = false;
bool initialPositionReached = false; //initially deploying fins after payload release?
bool finalPositionReached = false;

// ===== Files and Logging =====
File myFile;
int fileIndex = 0;

// ===== Servo Control =====
Servo s1, s2;
int c1, c2;

// ===== Program Constants =====
const int sens = 4;
const int fwd_spd = 110;
const int rvse_spd = 70;
const int s1_stop = 95;
const int s2_stop = 90;

// ===== PID Constants =====
float Kp = 0.7;
float Ki = 0.01;
float Kd = 0.5;
float prevError = 0;
float integral = 0;

// ===== Utilities =====
void log(const String &msg) {
  if (enableSerial) Serial.println(msg);
}

// ===== Rotary Encoder Read Function =====
uint16_t readAngle() {
  digitalWrite(ENC_CS_PIN, LOW);
  SPI.transfer16(0xFFFF);
  digitalWrite(ENC_CS_PIN, HIGH);

  delayMicroseconds(1);

  digitalWrite(ENC_CS_PIN, LOW);
  uint16_t result = SPI.transfer16(0xFFFF);
  digitalWrite(ENC_CS_PIN, HIGH);

  return result & 0x3FFF;
}

// ===== Count Existing Log Files =====
void countFiles() {
  fileIndex = 0;
  File root = SD.open("/");
  while (File entry = root.openNextFile()) {
    fileIndex++;
    entry.close();
  }
  root.close();
}

// ===== Count Existing Log Files =====
void initSD() {
  if (SD.begin(SD_CS_PIN))
  {
    log("SD init!");
    sdInitialized = true;
    countFiles();

    char filename[20];
    sprintf(filename, "log_%d.txt", fileIndex);
    myFile = SD.open(filename, FILE_WRITE);
    if (myFile)
    {
      myFile.println("===DATA START===");
      myFile.close();
      fileCreated = true;
      log("Writing to " + String(filename));
    }
    else
    {
      log("Failed to create file.");
    }
  } else {
    log("SD init failed.");
  }
}

// ===== Apogee Detection =====
void checkApogee(float ax, float ay, float az) {
  if (!imuAvailable) return;

  float mag = sqrt(ax * ax + ay * ay + az * az);

  if (!apogeeDetected) {
    if (mag > 0.5) wasRising = true;
    if (wasRising && mag < 0.5) {
      apogeeDetected = true;
      log("Apogee detected!");
    }
  }
}

// ===== Dummy Position Targeting =====
void dummyToPos(float currentAngle, int target) {
  if (currentAngle > 150) currentAngle -= 180;
  int mod = 1 - (1 / abs((int)currentAngle - target));

  if (currentAngle < target - sens) {
    c1 = fwd_spd * mod;
    c2 = fwd_spd * mod;
  } else if (currentAngle > target + sens) {
    c1 = rvse_spd * mod;
    c2 = rvse_spd * mod;
  } else {
    c1 = s1_stop;
    c2 = s2_stop;
  }
}

// ===== PID Controller =====
int runToPos(float currentAngle) {
  if (currentAngle < 35 || currentAngle > 85) return 90;

  float error = 90 - currentAngle;

  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  integral += error * 0.1;
  float derivative = (error - prevError) / 0.1;
  prevError = error;

  float pidOutput = Kp * error + Ki * integral + Kd * derivative;
  pidOutput = constrain(pidOutput, -90, 90);

  return constrain(90 - pidOutput, 0, 180);
}

// ===== Setup =====
void setup() {
  if (enableSerial) {
    Serial.begin(9600);
    while (!Serial.available());
    delay(100);
    log("SERIAL START");
  }

  if (!IMU.begin()) {
    log("Failed to initialize IMU!");
    imuAvailable = false;
  }

  pinMode(ENC_CS_PIN, OUTPUT);
  digitalWrite(ENC_CS_PIN, HIGH);
  log("Encoder init!");

  s1.attach(S1_PIN);
  s2.attach(S2_PIN);
  s1.write(s1_stop);
  s2.write(s2_stop);
  log("Servo init!");

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  if (enableLogging) {
    initSD();
  } 
}

// ===== Main Loop =====
void loop() {
  uint16_t angleRaw = readAngle();
  float angleDeg = angleRaw * 360.0 / 16384.0;

  // if (!initialPositionReached) {
  //   s1.write(s1_stop);
  //   s2.write(s2_stop);
  // } else if (apogeeDetected && !finalPositionReached) {
  //   dummyToPos(angleDeg, 75);
  // }

  s1.write(c1);
  s2.write(c2);

  float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
  if (imuAvailable && IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    checkApogee(accelX, accelY, accelZ);
  } else {
    imuAvailable = IMU.begin();
    delay(100);
  }

  // Prepare log message
  String buf = "ACC_X: " + String(accelX, 2)
             + " ACC_Y: " + String(accelY, 2)
             + " ACC_Z: " + String(accelZ, 2)
             + " | ENC: " + String(angleRaw)
             + " (" + String(angleDeg, 2) + " deg)"
             + " | Servo PWM: " + String(c1);

  log(buf);

  // Write to SD card
  if (enableLogging) {
    if (sdInitialized && fileCreated) {
      char filename[20];
      sprintf(filename, "log_%d.txt", fileIndex);
      myFile = SD.open(filename, FILE_WRITE);
      if (myFile) {
        myFile.println(buf);
        myFile.close();
      }
      else {
        log("File open failed.");
      }
    } else {
      initSD();
    }
  }

  delay(100);
}
