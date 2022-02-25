#include <Arduino_LSM9DS1.h> // this is for IMU on BLE 33
#include <Arduino_APDS9960.h>
#include <HIDMouse.h>
#include "Streaming.h"     // needed for the Serial output https://github.com/geneReeves/ArduinoStreaming
#include "SensorFusion.h" //SF

#define MOUSE_R_THRESH 400
#define MOUSE_L_THRESH 680

#define FirstArduino false

SF fusion;
HIDMouse bleMouse;

float PitchDiv=0.4, RollDiv=0.3, YawDiv=0.5;
// yaw / YawDiv -> position on the screen
// Mouse become less sensitive if YawDiv become higher.
// yaw -> X, roll -> Y

float gy, gz;
float angle[3]={0.0, 0.0, 0.0}; // pitch, roll, yaw
int offset[3]={0, 0, 0};  // pitch_offset, roll_offset, yaw_offset
float prev_ang[3]={0.0, 0.0, 0.0};  // previous final angle
float prev_Mahony_ang[3]={0.0, 0.0, 0.0};  // previous Mahony angle, calibrate 360
int pitch_name = 0, roll_name = 1, yaw_name = 2;  // Indicator of different angles
int Num_samp = 5;  // Number of samples to average before sending movement to mouse
int Num_cali = 5; // Number of offset calculation
volatile int8_t mouseX, mouseY, Wheel; // The coordinates of mouse
float gyroScale = 3.14159f / 155.0; // Scale gyro data
int mouseL=0, mouseR=0;
bool mouseLstat=0, mouseRstat=0;

void ori_esti();  // calculate the orientation
uint8_t angle2pos(int ang_nm); // change from angle to mouse position, change angle offset if needed
void mouse_cali();  // calibrate the mouse in the beginning
int detecting_gestures(); // detecting gestures and do corresponding response
void mouse_stablize();  // hold the mouse pointer on its current position
void mouse_click();     // detecting mouse click

void setup() {
  int Gesture=-1;
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  if (!IMU.begin()) 
    Serial.println("Error initializing IMU!");
  if (!APDS.begin()) 
    Serial.println("Error initializing APDS9960!");
  
  bleMouse.setDeviceName("Yuchen Mouse");
  bleMouse.begin();

  while(1) { 
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
  int i, tempX, tempY, gesture;
  float pitch_sum=0.0, roll_sum=0.0, yaw_sum=0.0, gz_sum=0.0, gy_sum=0.0, gy_avg, gz_avg;
  if(bleMouse.isConnected()) {
    for(i=0;i<Num_samp;i++) {
        ori_esti();
        pitch_sum += angle[pitch_name];
        roll_sum += angle[roll_name];
        yaw_sum += -angle[yaw_name];
        gz_sum += gz;
        gy_sum += gy;
    } 
    angle[pitch_name] = pitch_sum / float(Num_samp);
    angle[roll_name] = roll_sum / float(Num_samp);
    angle[yaw_name] = yaw_sum / float(Num_samp);
    gy_avg = gy_sum / float(Num_samp);
    gz_avg = gz_sum / float(Num_samp);

    // Serial << "gz_avg: " << (gz_avg*100) << "\t" << "gy_avg: " << (gy_avg*100) << endl;
    // Serial << "yaw: " << angle[2] << "\t" << "roll: " << angle[1] << endl;

    if (abs(gz_avg) < 0.05 && abs(gy_avg) < 0.05) {
      mouse_stablize();
    }
    else {
      prev_ang[2] = angle[2];
      prev_ang[1] = angle[1];
    }
      
    tempX = angle2pos(yaw_name);
    tempY = angle2pos(roll_name);

    if (tempX != mouseX || tempY != mouseY) {
      mouseX = tempX;
      mouseY = tempY;
      bleMouse.move(mouseX, mouseY);
    }
    
    gesture = detecting_gestures();
    if (gesture != -1) {
      bleMouse.move(mouseX, mouseY, Wheel);
      Wheel = 0;
    }

    Serial << "x: " << mouseX << "/y: " << mouseY << endl;

    mouse_click();
  }
  else {
    Serial << "BLE Mouse is not connected, Trying to reconnect." << endl;
    delay(1000);
  }
}

void mouse_cali() {
  int i=0;
  float p_sum=0.0, r_sum=0.0, y_sum=0.0;
  
  for(i=0;i<Num_cali;i++) {
    ori_esti();
    p_sum += angle[0];
    r_sum += angle[1];
    y_sum += -angle[2];
    Serial << i << ": " << angle[0] << "/" << angle[1] << "/" << angle[2] << endl;
  }
  offset[pitch_name] = int(p_sum / float(Num_cali) / PitchDiv) - 63;
  offset[roll_name] = int(r_sum / float(Num_cali) / RollDiv) - 63;
  offset[yaw_name] = int(y_sum / float(Num_cali) / YawDiv) - 63;
  Serial << "offset: " << offset[0] << "/" << offset[1] << "/" << offset[2] << endl;

  prev_Mahony_ang[1] = angle[1];
  prev_Mahony_ang[2] = angle[2];
}

void ori_esti() {
  float gx, ax, ay, az;
  float deltat;

  while (!( IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ));
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  
  gx = -gx * gyroScale;
  gy = -gy * gyroScale;
  gz = gz * gyroScale;

//  Serial << "g: " << (gx*1000) << "\t" << (gy*1000) << "\t" << (gz*1000) << "\t";
//  Serial << "a: " << (ax) << "\t" << (ay) << "\t" << (az) << "\t" << endl;

  //Calibration
  if (FirstArduino) { // First Arduino
    ax += 0.014;//0.014
    ay += 0.04;//0.02
    az += 0.005;
  
    gx += 0.009;
    gy -= 0.0025;
    gz -= 0.0045;
  }
  else {   //Second Arduino
    ax += 0.029;
    ay += 0.010;
    az += 0.005;
  
    gx += 0.013;
    gy -= 0.002;
    gz += 0.027;
  }

//  Serial << "g: " << (gz*100) << "\t" << (gy*100) << "\t";
//  Serial << "a: " << (ax*1000) << "\t" << (ay*1000) << "\t" << (az) << endl;
    
  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);
  
  angle[0] = fusion.getPitch();
  angle[1] = fusion.getRoll();
  angle[2] = fusion.getYaw();

  if ( abs(prev_Mahony_ang[1] - angle[1]) > 330 ) {
    if (prev_Mahony_ang[1] > angle[1])  // prev is 360 degrees, now is 0 degree
      offset[1] -= 360.0 / RollDiv;
    else
      offset[1] += 360.0 / RollDiv;
  }
  if ( abs(prev_Mahony_ang[2] - angle[2]) > 330 ) {
    if (prev_Mahony_ang[1] > angle[1])  // prev is 360 degrees, now is 0 degree
      offset[2] -= 360.0 / YawDiv;
    else
      offset[2] += 360.0 / YawDiv;
  }
  prev_Mahony_ang[1] = angle[1];
  prev_Mahony_ang[2] = angle[2];

//  Serial << angle[0] << "/" << angle[1] << "/" << angle[2] << endl;
//  Serial << angle[2] << "/" << angle[1] << endl;

  angle[0] = angle[0] / PitchDiv;
  angle[1] = angle[1] / RollDiv;
  angle[2] = angle[2] / YawDiv;    
}

/*  function angle2pos:
 *  1. mapping from angle to coordinates of mouse on screen
 *  2. change cooresponding angle offset when 
 *  (1) the cursor is out of the screen 
 *  (2) acc is calibrating the orientation slowly. 
 */
uint8_t angle2pos(int ang_nm) { 
  int pos;
  pos = int(angle[ang_nm]) - offset[ang_nm];
//  Serial << "function offset: " << ang_nm << "\t" << offset[ang_nm] << endl;
  if ( pos > 64 ) {
    offset[ang_nm] += pos - 64;
    return 127;
  }
  else if ( pos < -63 ) {
    offset[ang_nm] += pos + 63;
    return 0;
  }
  else
    return (pos+63);
}

int detecting_gestures() {
  int gesture = -1;
  if (APDS.gestureAvailable()) {
    gesture = APDS.readGesture();
    switch (gesture) {
      case GESTURE_UP:
        Serial.println("Detected RIGHT gesture");
        break;

      case GESTURE_DOWN:
        Serial.println("Detected LEFT gesture");
        break;

      case GESTURE_LEFT:
        Serial.println("Detected UP gesture");
        Wheel = -5; // Scroll down
        break;

      case GESTURE_RIGHT:
        Serial.println("Detected DOWN gesture");
        Wheel = 5;  // Scroll up
        break;

      default:
        // ignore
        break;
    }
  }
  return gesture;
}

void mouse_click() {
  int i;
  int numLoop = 5;
    mouseL = analogRead(A0);
    mouseR = analogRead(A1);

    if (mouseL < MOUSE_L_THRESH && !bleMouse.isPressed(1)) {
      for(i=0;i<numLoop;i++) {
        mouseL = analogRead(A0);
        if(mouseL >= MOUSE_L_THRESH) {
          break;
        }
      }
      if (i==numLoop) {
        bleMouse.press(1);
      } 
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
}

void mouse_stablize() { // stablize the mouse
  int add;
  if ( int(angle[2]) - int(prev_ang[2]) != 0) {
    // angle change is significant enough to influence the mouse position
    add = int(angle[2]) - int(prev_ang[2]);
    offset[2] += add;
    prev_ang[2] = angle[2];
//        Serial << "x_offset: " << offset[2] << "\t" << "x_angle: " << angle[2] << "\tadd\t" << add << endl;
  }
  if ( int(angle[1]) - int(prev_ang[1]) != 0) {
    // angle change is significant enough to influence the mouse position
    add = int(angle[1] - prev_ang[1]);
    offset[1] += int(angle[1]) - int(prev_ang[1]);
    prev_ang[1] = angle[1];
//        Serial << "y_offset: " << offset[1] << "\t" << "y_angle: " << angle[1] << "\tadd\t" << add << endl;
  }
}
