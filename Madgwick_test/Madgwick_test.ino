#include <Arduino_LSM9DS1.h>
#include "Madgwick.h"
#include <mbed.h>

mbed::Ticker tim1;

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
volatile bool flag = 0;

float ax, ay, az;
float gx, gy, gz;

void read_IMU() {
  if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
    }
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
    }
    flag = 1;
}

void setup() {
  Serial.begin(9600);

  // start the IMU and filter
  IMU.begin();
  filter.begin(104);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 104;
  microsPrevious = micros();
  tim1.attach(&read_IMU, 0.01);
}

void loop() {

    float roll, pitch, heading;
    if (flag) {

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
    flag = 0;
  }
}
