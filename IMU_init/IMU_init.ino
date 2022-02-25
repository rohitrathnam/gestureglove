//#include <Arduino_APDS9960.h>
#include <Arduino_LSM9DS1.h>

#include "Mahony.h"
#include "quat2euler.h"

extern volatile float q0, q1, q2, q3;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("Initialization done ...");
}
void loop() {
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  EulerAngles MyEuler;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(acc_x, acc_y, acc_z);
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
  }
//  Serial.print(gyro_x);Serial.print(gyro_y);Serial.println(gyro_z);
//  Serial.print(acc_x);Serial.print(acc_y);Serial.println(acc_z);
  MahonyAHRSupdateIMU(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);
  MyEuler = Eul_FromQuat({q0, q1, q2, q3}, EulOrdXYZs);
//  Serial.print("order: ");Serial.println(MyEuler.w);
//  Serial.print("x: ");Serial.println(MyEuler.x);
  Serial.print("y: ");Serial.println(MyEuler.y);
//  Serial.print("z: ");Serial.println(MyEuler.z);
}
