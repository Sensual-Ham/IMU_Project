#include "IMU.h"

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  IMU myIMU;

}

void loop() {
  // put your main code here, to run repeatedly:
  xAccel = myIMU.accelRead(0);
  yAccel = myIMU.accelRead(1);
  zAccel = myIMU.accelRead(2);
}
