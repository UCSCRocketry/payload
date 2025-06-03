#include <Arduino_LSM9DS1.h>
#include <Wire.h>

// I2C Slave Address
#define I2C_SLAVE_ADDRESS 0x10

const bool enableSerial = true;

// IMU Data Struct
struct IMUData
{
    float ax, ay, az; // Accelerometer
    float gx, gy, gz; // Gyroscope
};

class MovingAverageFilter
{
    float *buffer;
    int size, index;
    float sum;

public:
    MovingAverageFilter(int size = 5) : size(size), index(0), sum(0.0f)
    {
        buffer = new float[size]();
    }

    ~MovingAverageFilter()
    {
        delete[] buffer;
    }

    float filter(float value)
    {
        sum -= buffer[index];
        buffer[index] = value;
        sum += value;
        index = (index + 1) % size;
        return sum / size;
    }
};

MovingAverageFilter axFilt, ayFilt, azFilt;
MovingAverageFilter gxFilt, gyFilt, gzFilt;

IMUData imuData;

uint8_t *getIMUBytes()
{
    static uint8_t buf[sizeof(IMUData)];
    memcpy(buf, &imuData, sizeof(IMUData));
    return buf;
}

void onRequest()
{
    uint8_t *data = getIMUBytes();
    Wire.write(data, sizeof(IMUData));
}

void log(String msg)
{
    if (enableSerial)
    {
        Serial.println(msg);
    }
}

void setup()
{
    if (enableSerial) {
        Serial.begin(115200);
    }

    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onRequest(onRequest);

    if (!IMU.begin())
    {
        log("Failed to initialize IMU!");
        while (1)
            ;
    }

    log("IMU initialized, I2C ready");
}

void loop()
{
    float ax, ay, az;
    float gx, gy, gz;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable())
    {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        imuData.ax = axFilt.filter(ax);
        imuData.ay = ayFilt.filter(ay);
        imuData.az = azFilt.filter(az);

        imuData.gx = gxFilt.filter(gx);
        imuData.gy = gyFilt.filter(gy);
        imuData.gz = gzFilt.filter(gz);
    }

    delay(10);
}