#include "Arduino_BMI270_BMM150.h"

class MyBoschSensor: public BoschSensorClass {

  public:
    MyBoschSensor(TwoWire& wire = Wire) : BoschSensorClass(wire) {};

  protected:
    virtual int8_t configure_sensor(struct bmi2_dev *dev)
    {
      int8_t rslt;
      uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

      struct bmi2_int_pin_config int_pin_cfg;
      int_pin_cfg.pin_type = BMI2_INT1;
      int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
      int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
      int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
      int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
      int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

      struct bmi2_sens_config sens_cfg[2];
      sens_cfg[0].type = BMI2_ACCEL;
      sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
      sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_25HZ;
      sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
      sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
      sens_cfg[1].type = BMI2_GYRO;
      sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
      sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
      sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;
      sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
      sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

      rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_sensor_enable(sens_list, 2, dev);
      if (rslt != BMI2_OK)
        return rslt;

      return rslt;
    }
};

MyBoschSensor myIMU(Wire1);
int count;
float degreesX = 0;
float degreesY = 0;
int plusThreshold = 30, minusThreshold = -30;

void print_data() {
  // we can also read accelerometer / gyro data here!
  //Serial.println("Got new data!");
  //delay(1000);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  myIMU.debug(Serial);
  myIMU.onInterrupt(print_data);
  myIMU.begin();
  
  //set IMU to zero
  //angular velocity
  //PID loops - accel. velo. pos control.


  Serial.print("Accelerometer sample rate (Hz) = ");
  Serial.println(myIMU.accelerationSampleRate());
  Serial.println();
  Serial.print("Gyroscope sample rate (Hz) = ");
  Serial.println(myIMU.gyroscopeSampleRate());
  Serial.println();
  Serial.print("Accelerometer sample rate (Hz) = ");
  Serial.println(myIMU.accelerationSampleRate());
}

void loop() {
  count = 0;
  // put your main code here, to run repeatedly:
  float x, y, z, delta;

  delta = 0.02;

  if (myIMU.accelerationAvailable()) {
    myIMU.readAcceleration(x, y, z); //in 

    if(x > delta){
    x = 100*x;
    degreesX = map(x, 0, 97, 0, 90);
    Serial.print("Tilting up ");
    Serial.print(degreesX);
    Serial.println("  degrees");
    }
  if(x < -delta){
    x = 100*x;
    degreesX = map(x, 0, -100, 0, 90);
    Serial.print("Tilting down ");
    Serial.print(degreesX);
    Serial.println("  degrees");
    }
  if(y > delta){
    y = 100*y;
    degreesY = map(y, 0, 97, 0, 90);
    Serial.print("Tilting left ");
    Serial.print(degreesY);
    Serial.println("  degrees");
    }
  if(y < -delta){
    y = 100*y;
    degreesY = map(y, 0, -100, 0, 90);
    Serial.print("Tilting right ");
    Serial.print(degreesY);
    Serial.println("  degrees");
    }



    // Serial.print("accel: \t");
    // Serial.print(x);
    // Serial.print('\t');
    // Serial.print(y);
    // Serial.print('\t');
    // Serial.print(z);
    // Serial.println();
  }
  
  if (myIMU.gyroscopeAvailable()) {

    //raw rate of change of going up and down
    // accel xyz; gyro theta phi first rate of change

    myIMU.readGyroscope(x, y, z);

    if(y > plusThreshold)
    {
    Serial.println("Collision front");
    // delay(500);
    }
    if(y < minusThreshold)
    {
    Serial.println("Collision back");
    // delay(500);
    }
    if(x < minusThreshold)
    {
    Serial.println("Collision right");
    // delay(500);
    }
    if(x > plusThreshold)
    {
    Serial.println("Collision left");
    // delay(500);
    }

    // Serial.print("gyro: \t");
    // Serial.print(x);
    // Serial.print('\t');
    // Serial.print(y);
    // Serial.print('\t');
    // Serial.print(z);
    // Serial.println();
  }

  if (myIMU.magneticFieldAvailable()) {

    myIMU.readMagneticField(x, y, z);
    float ledvalue;

    if(x < 0)
    {
      ledvalue = -(x);
    }
    else{
      ledvalue = x;
    }
    
    analogWrite(LED_BUILTIN, ledvalue);
    // delay(500);

    // Serial.print("mag: \t");
    // Serial.print(x);
    // Serial.print('\t');
    // Serial.print(y);
    // Serial.print('\t');
    // Serial.print(z);
    // Serial.println();
  }
}
