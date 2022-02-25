// IMPORTANT
// this is not a working example, this is just to show how to set the library
// if you need a working example please see the other example


#include "SensorFusion.h" //SF
#include <Arduino_LSM9DS1.h>

SF fusion;

#define DEG_RAD_FACTOR 0.01745329

float raw_mx, raw_my, raw_mz;
float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

float accel_zerog[] = {0, 0, 0};
float gyro_zerorate[] = {0, 0, 0};
float mag_hardiron[] = {-4.24, 3.63, 10.12};
float mag_softiron[] = {0.984, 0.034, 0.006, 0.034, 1.011, 0.002, 0.006, 0.002, 1.007};

void setup() {
  Serial.begin(9600); //serial to display data
  IMU.begin();
  // your IMU begin code goes here
}

void loop() {

  // now you should read the gyroscope, accelerometer (and magnetometer if you have it also)
  // NOTE: the gyroscope data have to be in radians
  // if you have them in degree convert them with: DEG_TO_RAD example: gx * DEG_TO_RAD
  while (!( IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()));
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  IMU.readMagneticField(raw_mx, raw_my, raw_mz);

//  Serial.print(ax); Serial.print(" ");
//  Serial.print(ay); Serial.print(" ");
//  Serial.print(az); Serial.print(" ");
//  Serial.print(gx); Serial.print(" ");
//  Serial.print(gy); Serial.print(" ");
//  Serial.print(gz); Serial.print(" ");
//  Serial.print(mx); Serial.print(" ");
//  Serial.print(my); Serial.print(" ");
//  Serial.println(mz);

  calibrate();
  gx = gx * DEG_RAD_FACTOR;
  gy = gy * DEG_RAD_FACTOR;
  gz = gz * DEG_RAD_FACTOR;
  
  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  //choose only one of these two:
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate

  pitch = fusion.getPitch();
  roll = fusion.getRoll();    //you could also use getRollRadians() ecc
  yaw = fusion.getYaw();

  Serial.print("Pitch:\t"); Serial.println(pitch);
  Serial.print("Roll:\t"); Serial.println(roll);
  Serial.print("Yaw:\t"); Serial.println(yaw);
  Serial.println();
}

void calibrate() {
    // hard iron cal
    raw_mx = raw_mx - mag_hardiron[0];
    raw_my = raw_my - mag_hardiron[1];
    raw_mz = raw_mz - mag_hardiron[2];
    // soft iron cal
    mx = raw_mx * mag_softiron[0] + raw_my * mag_softiron[1] + raw_mz * mag_softiron[2];
    my = raw_mx * mag_softiron[3] + raw_my * mag_softiron[4] + raw_mz * mag_softiron[5];
    mz = raw_mx * mag_softiron[6] + raw_my * mag_softiron[7] + raw_mz * mag_softiron[8];
    // gyro cal
    gx -= gyro_zerorate[0];
    gy -= gyro_zerorate[1];
    gz -= gyro_zerorate[2];
    // acc cal
    ax -= accel_zerog[0];
    ay -= accel_zerog[1];
    az -= accel_zerog[2];
}
