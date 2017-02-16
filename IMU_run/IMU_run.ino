#include "IMU.h"
#include "MatrixMath.h"
#include <Math.h>
#define RAD2DEG 57.2958
#define GYROSENSITIVITY 14.375

float xAccel, yAccel, zAccel;
float xGyro, yGyro, zGyro;
double Pitch, Roll;
IMU myIMU;
MatrixMath MM;

// Calibration values
float xAccel_calib, yAccel_calib, zAccel_calib;
float xGyro_calib, yGyro_calib, zGyro_calib;

// Matrices for Kalman Filter
float I[(2)][(2)];

// Error covariance for roll and pitch
float P_roll[(2)][(2)];
float P_pitch[(2)][(2)];

// State vector for roll and pitch
float x_roll[(2)];
float x_pitch[(2)];

// State transition matrix
float A[(2)][(2)];

// Keep track of time
float dt;
unsigned long time_prev;

// Noise Covariance
float Q_angle;
float Q_gyro;
float R;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myIMU.init();

  // Run calibration routine
  calibrate();


  // Matrices for Kalman Filter
  // Identity Matrix:
  I[0][0] = 1;
  I[1][0] = 0;
  I[0][1] = 0;
  I[1][1] = 1;

  // Initial P matrices
  P_roll[0][0] = 0;
  P_roll[1][0] = 0;
  P_roll[0][1] = 0;
  P_roll[1][1] = 0;
  P_pitch[0][0] = 0;
  P_pitch[1][0] = 0;
  P_pitch[0][1] = 0;
  P_pitch[1][1] = 0;

  // Initial state vector
  x_roll[0] = 0;
  x_roll[1] = 0;
  x_pitch[0] = 0;
  x_pitch[1] = 0;

  // Initial noise values
  Q_angle = 0.001;
  Q_gyro = 0.003;
  R = 0.03;

  // Initial time measurement
  time_prev = millis();
}

void loop() {
  // Get raw accelerometer and gyro data
  // TODO: Implement filters for data
  xAccel = (myIMU.accelRead(0) - xAccel_calib) / 1000;
  yAccel = (myIMU.accelRead(1) - yAccel_calib) / 1000;
  zAccel = (myIMU.accelRead(2) - zAccel_calib) / 1000;
  xGyro = (myIMU.gyroRead(0) - xGyro_calib) / GYROSENSITIVITY;
  yGyro = (myIMU.gyroRead(1) - yGyro_calib) / GYROSENSITIVITY;
  zGyro = (myIMU.gyroRead(2) - zGyro_calib) / GYROSENSITIVITY;

  //  xGyro = 0;
  //  yGyro = 0;
  //  zGyro = 0;

  // Print raw data to serial
//  Serial.print("Accelerometer x: ");  Serial.println(xAccel);
//  Serial.print("Accelerometer y :");  Serial.println(yAccel);
//  Serial.print("Accelerometer z: ");  Serial.println(zAccel);

//  Serial.print("Gyro x: "); Serial.println(xGyro);
//  Serial.print("Gyro y: "); Serial.println(yGyro);
//  Serial.print("Gyro Z: "); Serial.println(zGyro);

  // Calculate pitch and roll from accelerometer data
  // Roll: Around x-axis
  // Pitch: Around y-axis

  Pitch = 2 * atan2(xAccel , sqrt((xAccel * xAccel) + (zAccel * zAccel)) );
  Roll = 2 * atan2(yAccel , sqrt((yAccel * yAccel) + (zAccel * zAccel)) );
//  Serial.print("Accelerometer Pitch: "); Serial.println(Pitch * RAD2DEG);
//  Serial.print("Roll: "); Serial.println(Roll * RAD2DEG);


  // Calculate using Kalman filter

  dt = ((float)millis() - (float)time_prev) / 1000;
  time_prev = millis();
//  Serial.print("dt: ");  Serial.println(dt);
  kalman((float*)P_pitch, (float*)x_pitch, Pitch, xGyro, dt);
  Serial.print("Kalman Pitch ");
  Serial.println(x_pitch[0] * RAD2DEG);
//  Serial.print("Drift ");
//  Serial.println(x_pitch[1] * RAD2DEG);

  Serial.println( );
  delay(10);
}

void kalman(float *P, float *x, float angle_measured, float rate_measured, float dT) {
  // Step 2
  // Estimate elements of state vector
  float rate_estimated;
  float angle_estimated;
  rate_estimated = rate_measured - x[1];
  angle_estimated =  dT * rate_estimated + x[0];
  
//  Serial.print("dT: ");  Serial.println(dT);
//  Serial.print("x[0] initial: ");  Serial.println(x[0]);
//  Serial.print("x[1] initial: ");  Serial.println(x[1]);
//
//  Serial.print("rate_estimated: "); Serial.println(rate_estimated);
//  Serial.print("angle_estimated: "); Serial.println(angle_estimated);


  // Step 3
  P[0] -= dT * ( P[2] + P[1] - (dT * P[3])  - Q_angle);
  P[1] -= dT * P[3];
  P[2] -= dT * P[3];
  P[3] += Q_gyro * dT;

//  MM.Print((float*)P,  2,  2, "P");
  //  Serial.print("P[0]: ");  Serial.println(P[0]);
  //  Serial.print("P[1]: ");  Serial.println(P[1]);
  //  Serial.print("P[2]: ");  Serial.println(P[2]);
  //  Serial.print("P[3]: ");  Serial.println(P[3]);

  // Step 4
  float K[2];
  float S = P[0] + R;
  K[0] = P[0] / S;
  K[1] = P[1] / S;
//  MM.Print((float*)K,  2,  1, "K");
  //  Serial.print("K[0]: ");  Serial.println(K[0]);
  //  Serial.print("K[1]: ");  Serial.println(K[1]);

  // Step 5
  x[0] += K[0] * (angle_measured - angle_estimated);
  x[1] += K[1] * (angle_measured - angle_estimated);

//  Serial.print("x[0] final: ");  Serial.println(x[0]);
//  Serial.print("x[1] final: ");  Serial.println(x[1]);

  // Step 6
  P[3] -= P[1] * K[1];
  P[2] -= P[0] * K[1];
  P[1] -= P[1] * K[0];
  P[0] -= P[0] * K[0];
}

void calibrate () {
  Serial.println("Beginning calibration routine...");
  xAccel = 0; yAccel = 0; zAccel = 0;
  xGyro = 0; yGyro = 0; zGyro = 0;
  int calibIters = 1000;
  for ( int i = 0; i < calibIters; i++) {
    xAccel += myIMU.accelRead(0);
    yAccel += myIMU.accelRead(1);
    //zAccel += myIMU.accelRead(2);
    xGyro += myIMU.gyroRead(0);
    yGyro += myIMU.gyroRead(1);
    zGyro += myIMU.gyroRead(2);
//    Serial.print(xAccel); Serial.println(yAccel);
//    Serial.print(xGyro); Serial.print(yGyro); Serial.println(zGyro);
//    Serial.println();
    delay(5);
  }

  xAccel_calib = xAccel/calibIters;
  yAccel_calib = yAccel/calibIters;
  //zAccel_calib = zAccel_calib;
  xGyro_calib = xGyro/calibIters;
  yGyro_calib = yGyro/calibIters;
  zGyro_calib = zGyro/calibIters;
//  Serial.println("Calibration Results:");
//  Serial.print(xAccel_calib); Serial.println(yAccel_calib);
//  Serial.print(xGyro); Serial.print(yGyro); Serial.println(zGyro);

  Serial.println("Calibration routine complete.");
}

