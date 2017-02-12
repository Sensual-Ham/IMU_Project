#include "IMU.h"
#include "MatrixMath.h"
#include <Math.h>
#define RAD2DEG 57.2958

float xAccel, yAccel, zAccel;
float xGyro, yGyro, zGyro;
double Pitch, Roll; 
IMU myIMU;
MatrixMath Matrix;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myIMU.init();

}

void loop() {
  // Get raw accelerometer and gyro data
  // TODO: Implement filters for data
  xAccel = myIMU.accelRead(0) / 1000;
  yAccel = myIMU.accelRead(1) / 1000;
  zAccel = myIMU.accelRead(2) / 1000;
  xGyro = myIMU.gyroRead(0);
  yGyro = myIMU.gyroRead(1);
  zGyro = myIMU.gyroRead(2);

  // Print raw data to serial 
  Serial.print("Accelerometer x: ");  Serial.println(xAccel);
  Serial.print("Accelerometer y :");  Serial.println(yAccel);
  Serial.print("Accelerometer z: ");  Serial.println(zAccel);

  Serial.print("Gyro x: "); Serial.println(xGyro);
  Serial.print("Gyro y: "); Serial.println(yGyro);
  Serial.print("Gyro Z: "); Serial.println(zGyro);

  // Calculate pitch and roll from accelerometer data
  // Roll: Around x-axis
  // Pitch: Around y-axis
  
  Pitch = 2 * atan2(xAccel ,sqrt((xAccel*xAccel) + (zAccel*zAccel)) );
  Roll = 2 * atan2(yAccel ,sqrt((yAccel*yAccel) + (zAccel*zAccel)) );
  Serial.print("Pitch: "); Serial.println(Pitch * RAD2DEG); 
  Serial.print("Roll: "); Serial.println(Roll * RAD2DEG);
  Serial.println( );
  
  delay(1000);
}

