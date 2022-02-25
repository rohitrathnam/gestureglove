#include <HIDMouse.h>
#include <Arduino_LSM9DS1.h>
#include "Kalman.h"

#define MODE 1

HIDMouse bleMouse;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only

double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t disp_counter = 0;

bool filter_ready = 0;

const int8_t RANGE[9] = {0, 5, 15, 25, 35, 45, 55, 65, 75};
const int8_t STEP[8] = {0, 2, 5, 8, 12, 16, 22, 26};

int8_t mouseX, mouseY;

int sensorPin = A0;
int sensorPin1 = A1; // select the input pin for the potentiometer
int sensorValue = 0;
int sum = 0;
int sum1 = 0;
float avg;
float avg1;
int a[10];
int b[10];// variable to store the value coming from the sensor
int j;

void setup() {

  Serial.begin(9600);

  if (!IMU.begin()) {
    while (1);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(sensorPin, INPUT);
  pinMode(sensorPin1, INPUT);

  bleMouse.setDeviceName("Gesture glove");
  bleMouse.setManufacturerName("Manufacturer");
  bleMouse.setBatteryLevel(69);
  bleMouse.begin();

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (!IMU.accelerationAvailable());
  IMU.readAcceleration(accX, accY, accZ);
  while (!IMU.gyroscopeAvailable());
  IMU.readGyroscope(gyroX, gyroY, gyroZ);

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;

  timer = micros();
}

void loop() {


  while (bleMouse.isConnected()) {

    digitalWrite(LED_BUILTIN, HIGH);

    while (!IMU.accelerationAvailable());
    IMU.readAcceleration(accX, accY, accZ);
    while (!IMU.gyroscopeAvailable());
    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    if (timer > 0.5) {
      timer = micros();
      while (!IMU.accelerationAvailable());
      IMU.readAcceleration(accX, accY, accZ);
      while (!IMU.gyroscopeAvailable());
      IMU.readGyroscope(gyroX, gyroY, gyroZ);
      delay(0.001);
      double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    }
    else {
      timer = micros();
    }

    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    double gyroXrate = gyroX;
    double gyroYrate = gyroY;

    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;

    if (j == 10)
    { j = 0;
    }
    a[j] = analogRead(sensorPin);
    b[j] = analogRead(sensorPin1);
    j += 1;

    sum = 0;
    sum1 = 0;
    // Serial.println(sensorValue);
    for (int i = 0; i < 10; i++)
    {
      sum = sum + a[i];
      sum1 = sum1 + b[i];
    }
    avg = sum / 10.0;
    avg1 = sum1 / 10.0;

    // Send mouse move every 10 samples
    if (disp_counter % 2 == 0) {
      disp_counter = 1;
      Serial.print(kalAngleX); Serial.print("\t");
      Serial.print(kalAngleY); Serial.print("\t");
      Serial.println(dt);
      for (uint8_t i = 0; i < 8; i++) {
        if (kalAngleX >= RANGE[i] && kalAngleX <= RANGE[i + 1]) {
          mouseY = STEP[i];
          break;
        }
        else if (kalAngleX >= -1 * RANGE[i + 1] && kalAngleX <= -1 * RANGE[i]) {
          mouseY = -1 * STEP[i];
          break;
        }
      }
      for (uint8_t i = 0; i < 8; i++) {
        if (kalAngleY >= RANGE[i] && kalAngleY <= RANGE[i + 1]) {
          mouseX = STEP[i];
          break;
        }
        else if (kalAngleY >= -1 * RANGE[i + 1] && kalAngleY <= -1 * RANGE[i]) {
          mouseX = -1 * STEP[i];
          break;
        }
      }

      bleMouse.move(mouseX, mouseY);
      if (avg1 < 650 && !bleMouse.isPressed(1)) {
        bleMouse.press(1);
        Serial.println("LEFT");
      }
      else {
        if (bleMouse.isPressed(1)) {
          bleMouse.release(1);
        }
      }
      if (avg > 500 && !bleMouse.isPressed(2)) {
        bleMouse.press(2);
        Serial.println("RIGHT");
      }
      else {
        if (bleMouse.isPressed(2)) {
          bleMouse.release(2);
        }
      }
    }
    else {
      disp_counter++;
    }
    //bleMouse.move(0, yAcc); // y-Axis down

    //}
    digitalWrite(LED_BUILTIN, LOW);
  }
}
