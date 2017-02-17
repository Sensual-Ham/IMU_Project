#include "IMU.h"
#include "MatrixMath.h"
#include <Math.h>
#define RAD2DEG 57.2958

float xAccel, yAccel, zAccel;
float xGyro, yGyro, zGyro;
double Pitch, Roll;
IMU myIMU;
MatrixMath MM;

// Calibration values
float xAccel_calib, yAccel_calib, zAccel_calib;
float xGyro_calib, yGyro_calib, zGyro_calib;

// Matrices for Kalman Filter
float I[(3)][(3)];

float A[(3)][(3)];
float H[2][3];
float H_T[3][2];

// Error covariance for roll and pitch
float P_roll[(2)][(2)];
float P_pitch[(2)][(2)];

// State vector for roll and pitch
float x_roll[(3)];
float x_pitch[(3)];



// Keep track of time
float dt;
unsigned long time_prev;

// Noise Covariance
float Q_angle;
float Q_gyro;
float R[2];

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

  // Set state transition matrix
  A[0][0] = 1;
  A[1][0] = 0;
  A[1][1] = 1;
  A[1][2] = -1;
  A[2][0] = 0;
  A[2][1] = 0;
  A[2][2] = 1;

  // Set measurement matrix
  H[0][0] = 1;
  H[1][1] = 1;
  H_T[0][0] = 1;
  H_T[1][1] = 1;

  // Noise values
  Q_angle = 0.001;
  Q_gyro = 0.003;
  R[0] = 0.03;
  R[1] = 0.03;

  // Initial time measurement
  time_prev = millis();
}

void loop() {
  // Get raw accelerometer and gyro data
  // TODO: Implement filters for data
  xAccel = (myIMU.accelRead(0) - xAccel_calib) / 1000;
  yAccel = (myIMU.accelRead(1) - yAccel_calib) / 1000;
  zAccel = (myIMU.accelRead(2) - zAccel_calib) / 1000;
  xGyro = (myIMU.gyroRead(0) - xGyro_calib);
  yGyro = (myIMU.gyroRead(1) - yGyro_calib);
  zGyro = (myIMU.gyroRead(2) - zGyro_calib);

  // Print raw data to serial
  //  Serial.print("Accelerometer x: ");  Serial.println(xAccel);
  //  Serial.print("Accelerometer y :");  Serial.println(yAccel);
  //  Serial.print("Accelerometer z: ");  Serial.println(zAccel);

  //  Serial.print("Gyro x: "); Serial.println(xGyro);
  Serial.print("Gyro y: "); Serial.println(yGyro);
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

  A[0][1] = dt;
  A[0][2] = dt;
  //  Serial.print("dt: ");  Serial.println(dt);
  //kalman((float*)P_pitch, (float*)x_pitch, Pitch, xGyro, dt);
  //  Serial.print("Kalman Pitch ");
  //  Serial.println(x_pitch[0] * RAD2DEG);
  //  Serial.print("Drift ");
  //  Serial.println(x_pitch[1] * RAD2DEG);

  Serial.println( );
  delay(10);
}

void kalman(float *P, float *x, float angle_measured, float rate_measured, float dT) {
  // Set up variables for use
  float rate_estimated;
  float angle_estimated;
  float x_previous[3];
  float x_predicted[3];
  float A_transpose[3][3];
  float P_predicted[3][3];
  float P_old[3][3];
  

  MM.Copy(x, 3, 1, (float*) x_previous);
  MM.Copy(P, 3, 3, (float*) P_old);

  // Put dT in A matrix
  A[0][1] = dT;
  A[0][2] = -dT;
  MM.Transpose((float*) A, 3, 3, (float*) A_transpose);

  // Step 2: Estimate a posteriori state estimate
  MM.Multiply((float*) A, (float*) x, 3, 3, 1, (float*) x_predicted);

  // Step 3: Prediction error covariance
  {
    float multTemp[3][3];
    MM.Multiply((float*) A, (float*) P_old, 3, 3, 3, (float*) multTemp);
    MM.Multiply((float*) multTemp, (float*) A_transpose, 3, 3, 3, (float*) P_predicted);
  }

  // Add process noise matrix
  //  P_predicted[0][0] += 0 * dT;
  //  P_predicted[1][1] += 0 * dT;
  //  P_predicted[2][2] += 0 * dT;

  // Step 4: Kalman gain calculation
  float S[2][2];
  float K[3][2];
  {
    float temp1[2][3];
    MM.Multiply((float*) H, (float*) P_predicted, 2, 3, 3, (float*) temp1);
    MM.Multiply((float*) temp1, (float*) H_T, 2, 3, 2, (float*) S);
  }
  S[0][0] += R[0];
  S[1][1] += R[1];
  MM.Invert(float* S, int 2);]

  {
    float temp[3][2];
    MM.Multiply((float*) P, (float*) H_T, 3, 3, 2, (float*) temp);
    MM.Multiply((float*) temp2, (float*) S, 3, 2, 2, (float*) K);
  }

  // Step 5: New state vector calculation
  {
    float z[2];
    float residual[2];
    float temp[3];
    z[0] = angle_measured;
    z[2] = rate_measured;
    float predicted_measurement[2];
    MM.Multiply((float*) H, (float*) x_predicted, 2, 3, 1, (float*) predicted_measurement);
    MM.Subtract(float* z, float* predicted_measurement, int 2, int 1, float* residual);
    MM.Multiply((float*) K, (float*) residual, 3, 2, 1, (float*) temp);
    MM.Add(float* x_predicted, float* temp3, int 3, int 1, float* x);

    x[0] += x_predicted[0];
    x[1] += x_predicted[1];
    x[2] += x_predicted[2];
  }

  // Step 6: Update P Matrix
  {
    float P_old[3][3];
    MM.Copy(float* P, int 3, int 3, float* P_old);
    float temp[3][3];
    MM.Multiply((float*) K, (float*) H, 3, 2, 3, (float*) temp);
    float temp2[3][3];
    
  }

  
  
  //  K[0] = K[0]/(P_predicted[0]0] + R);
  //  K[1] = K[1]/(P_predicted[0]0] + R);
  //  K[2] = K[2]/(P_predicted[0]0] + R);

  // Step 5: New state estimation


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
    zAccel += myIMU.accelRead(2);
    xGyro += myIMU.gyroRead(0);
    yGyro += myIMU.gyroRead(1);
    zGyro += myIMU.gyroRead(2);
    //    Serial.print(xAccel); Serial.println(yAccel);
    //    Serial.print(xGyro); Serial.print(yGyro); Serial.println(zGyro);
    //    Serial.println();
    delay(5);
  }

  xAccel_calib = xAccel / calibIters;
  yAccel_calib = yAccel / calibIters;
  zAccel_calib = (zAccel_calib / calibIters) - 1;
  xGyro_calib = xGyro / calibIters;
  yGyro_calib = yGyro / calibIters;
  zGyro_calib = zGyro / calibIters;
  //  Serial.println("Calibration Results:");
  //  Serial.print(xAccel_calib); Serial.println(yAccel_calib);
  //  Serial.print(xGyro); Serial.print(yGyro); Serial.println(zGyro);

  Serial.println("Calibration routine complete.");
}

