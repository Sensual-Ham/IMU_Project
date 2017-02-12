#include "IMU.h"
#include "MatrixMath.h"

float xAccel, yAccel, zAccel;
float xGyro, yGyro, zGyro;
IMU myIMU;
MatrixMath Matrix;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myIMU.init();

}

void loop() {
  // get & print accelerometer data
  Serial.print("Accelerometer x: ");
  xAccel = myIMU.accelRead(0);
  Serial.println(xAccel);
  Serial.print("Accelerometer y :");
  yAccel = myIMU.accelRead(1);
  Serial.println(yAccel);
  Serial.print("Accelerometer z: ");
  zAccel = myIMU.accelRead(2);
  Serial.println(zAccel);

  //Get & print gyro values
  Serial.print("Gyro x: ");
  xGyro = myIMU.gyroRead(0);
  Serial.println(xGyro);
  Serial.print("Gyro y: ");
  yGyro = myIMU.gyroRead(1);
  Serial.println(yGyro);
  Serial.print("Gyro Z: ");
  zGyro = myIMU.gyroRead(2);
  Serial.println(zGyro);
  Serial.println( );
  delay(1000);
}


