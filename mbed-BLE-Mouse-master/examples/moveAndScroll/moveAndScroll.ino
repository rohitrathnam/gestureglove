/**
 * This example turns the nano 33 BLE into a Bluetooth LE mouse that continuously moves the mouse.
 */
#include <HIDMouse.h>

HIDMouse bleMouse;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  bleMouse.setDeviceName("THE GESTURE GLOVE");
  bleMouse.setManufacturerName("Manufacturer");
  bleMouse.setBatteryLevel(69);
  bleMouse.begin();
}

void loop() {
  if(bleMouse.isConnected()) {

    digitalWrite(LED_BUILTIN, HIGH);
    unsigned long startTime;

    startTime = millis();
    while(millis()<startTime+2000) {
      bleMouse.move(0,0,1);
      delay(100);
    }
    delay(500);
    startTime = millis();
    while(millis()<startTime+2000) {
      bleMouse.move(0,0,-1);
      delay(100);
    }
    delay(500);

    startTime = millis();
    while(millis()<startTime+2000) {
      bleMouse.move(0,0,-1);
      delay(100);
    }
    delay(500);

    startTime = millis();
    while(millis()<startTime+2000) {
      bleMouse.move(0,0,1);
      delay(100);
    }
    delay(500);

    startTime = millis();
    while(millis()<startTime+2000) {
      bleMouse.move(0,-10);
      delay(100);
    }
    delay(500);

    startTime = millis();
    while(millis()<startTime+2000) {
      bleMouse.move(0,20);
      delay(100);
    }
    delay(500);

    startTime = millis();
    while(millis()<startTime+2000) {
      bleMouse.move(-30,0);
      delay(100);
    }
    delay(500);

    startTime = millis();
    while(millis()<startTime+2000) {
      bleMouse.move(40,0);
      delay(100);
    }
    delay(500);

  }
  digitalWrite(LED_BUILTIN, LOW);
}
