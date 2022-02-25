#include <Arduino_LSM9DS1.h>

#define SENSORS_RADS_TO_DPS 0.0174533

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  IMU.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  while (!( IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()));
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  IMU.readMagneticField(mx, my, mz);
  Serial.print("Raw:");
  Serial.print(int(ax*8192/9.8)); Serial.print(",");
  Serial.print(int(ay*8192/9.8)); Serial.print(",");
  Serial.print(int(az*8192/9.8)); Serial.print(",");
  Serial.print(int(gx*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(gy*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(gz*SENSORS_RADS_TO_DPS*16)); Serial.print(",");
  Serial.print(int(mx*10)); Serial.print(",");
  Serial.print(int(my*10)); Serial.print(",");
  Serial.println(int(mz*10));
}
