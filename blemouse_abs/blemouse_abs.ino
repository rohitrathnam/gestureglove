#include <Arduino_LSM9DS1.h> . // this is for IMU on BLE 33
#include <Arduino_APDS9960.h>
#include <HIDMouse.h>
#include "Streaming.h"     // needed for the Serial output https://github.com/geneReeves/ArduinoStreaming
#include "SensorFusion.h" //SF

#define MOUSE_R_THRESH 400
#define MOUSE_L_THRESH 680
#define DEBUG 0

SF fusion;
HIDMouse bleMouse;

//float pitch, roll, yaw;
//float prev_pitch, prev_roll, prev_yaw;
//int pitch_offset, roll_offset, yaw_offset;
float angle[3] = {0.0, 0.0, 0.0}; // pitch, roll, yaw
int offset[3] = {0.0, 0.0, 0.0}; // pitch_offset, roll_offset, yaw_offset
float prev_ang[3] = {0.0, 0.0, 0.0};
int pitch_name = 0, roll_name = 1, yaw_name = 2;  // Indicator of different angles
int Num_samp = 5;  // Number of samples to average before sending movement to mouse
int Num_cali = 5; // Number of ooffset calculation
uint8_t mouseX, mouseY; // The coordinates of mouse
float gyroScale = 3.14159f / 155.0; // Scale gyro data
int mouseL = 0, mouseR = 0;
int counter = 0;

void ori_esti();
uint8_t angle2pos(int ang_nm); // change from angle to mouse position, change angle offset if needed
void mouse_cali();
int detecting_gestures();

void setup() {
  int Gesture = -1;
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  if (!IMU.begin())
    Serial.println("Error initializing IMU!");
  if (!APDS.begin())
    Serial.println("Error initializing APDS9960!");

  bleMouse.setDeviceName("Gesture_glove");
  bleMouse.begin();

  while (1) {
    Gesture = detecting_gestures();
    if ( Gesture == -1 ) {
      Serial << "Detecting gestures..." << endl;
    }
    else {
      Serial << "Gesture is " << Gesture << endl;
      break;
    }
  }

  mouse_cali();
}

void loop() {
  int i, tempX, tempY;
  float pitch_sum = 0.0, roll_sum = 0.0, yaw_sum = 0.0;
  if (bleMouse.isConnected() || DEBUG) {
    for (i = 0; i < Num_samp; i++) {
      ori_esti();
      //      Serial << angle[0] << "/" << angle[1] << "/" << angle[2] << endl;
      pitch_sum += angle[0];
      roll_sum += angle[1];
      yaw_sum += -angle[2];
    }
    angle[0] = pitch_sum / 2.0;
    angle[1] = roll_sum / 2.0;
    angle[2] = yaw_sum / 2.0;
    Serial << "angles: " << angle[0] << "/" << angle[1] << "/" << angle[2];

    tempX = angle2pos(yaw_name);
    tempY = angle2pos(roll_name);

    mouseL = analogRead(A1);
    mouseR = analogRead(A0);

    if ((tempX != mouseX) || (tempY != mouseY)) {
      mouseX = tempX;
      mouseY = tempY;
      bleMouse.move(mouseX, mouseY);
    }

    if (mouseL < MOUSE_L_THRESH && !bleMouse.isPressed(1)) {
      bleMouse.press(1);
    }
    else if (mouseL > MOUSE_L_THRESH && bleMouse.isPressed(1)) {
      bleMouse.release(1);
    }
    if (mouseR > MOUSE_R_THRESH && !bleMouse.isPressed(2)) {
      bleMouse.press(2);
    }
    else if (mouseR < MOUSE_R_THRESH && bleMouse.isPressed(2)) {
      bleMouse.release(2);
    }

    Serial << "x: " << mouseX << "/y: " << mouseY << " ";
    Serial.print(bleMouse.isPressed(1));
    Serial.print(" ");
    Serial.println(bleMouse.isPressed(2));
  }
  else {
    Serial << "BLE Mouse is not connected, Trying to reconnect." << endl;
    delay(1000);
  }
}

void mouse_cali() {
  int i = 0;
  float p_sum, r_sum, y_sum;

  for (i = 0; i < Num_cali; i++) {
    ori_esti();
    p_sum += angle[0];
    r_sum += angle[1];
    y_sum += -angle[2];
    Serial << i << ": " << angle[0] << "/" << angle[1] << "/" << angle[2] << endl;
  }
  offset[pitch_name] = int(p_sum / Num_cali);
  offset[roll_name] = int(r_sum / Num_cali);
  offset[yaw_name] = int(y_sum / Num_cali);

  Serial << "offset: " << offset[0] << "/" << offset[1] << "/" << offset[2] << endl;
}

void ori_esti() {
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  float deltat;
  while (!( IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()));

  IMU.readAcceleration(ax, ay, az);

  IMU.readGyroscope(gx, gy, gz);
  IMU.readMagneticField(mx, my, mz);
  if (gx < 1 && gx > -1) gx = 0;
  if (gy < 1 && gy > -1) gy = 0;
  if (gz < 1 && gz > -1) gz = 0;

  gx = -gx * gyroScale;
  gy = -gy * gyroScale;
  gz = gz * gyroScale;

  //Calibration
  ax += 0.014;//0.014
  ay += 0.04;//0.02
  az += 0.005;

  //  gx += 0.009;
  //  gy -= 0.0025;
  //  gz -= 0.0045;

  //Serial << "g: " << abs(gx*1000) << "/" << abs(gy*1000) << "/" << abs(gz*1000);

  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);

  angle[0] = fusion.getPitch();
  angle[1] = fusion.getRoll();
  angle[2] = fusion.getYaw();

  //  if (abs(gz) < 10.0) {
  //    offset[2] += int(angle[2] - prev_ang[2]);
  //  }
  //  if (abs(gy) < 10.0) {
  //    offset[1] += int(angle[1] - prev_ang[1]);
  //  }
  prev_ang[2] = angle[2];
  prev_ang[1] = angle[1];
}

/*  function angle2pos:
    1. mapping from angle to coordinates of mouse on screen
    2. change cooresponding angle offset when
    (1) the cursor is out of the screen
    (2) acc is calibrating the orientation slowly.
*/
uint8_t angle2pos(int ang_nm) {
  int pos;
  pos = (int(angle[ang_nm]) - offset[ang_nm]);
  if ( pos > 64 ) {
    offset[ang_nm] += pos - 64;
    return 127;
  }
  else if ( pos < -63 ) {
    offset[ang_nm] += pos + 63;
    return 0;
  }
  else
    return (pos + 63);
}

int detecting_gestures() {
  int gesture = -1;
  if (APDS.gestureAvailable()) {
    gesture = APDS.readGesture();
  }
  return gesture;
}
