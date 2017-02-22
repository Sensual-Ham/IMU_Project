#include "IMU.h"
#include "MatrixMath.h"
#include <Math.h>
#define RAD2DEG 57.2958

bool Kalman_verbose;
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
float A_transpose[3][3];
float H[2][3];
float H_T[3][2];

// Error covariance for roll and pitch
float P_roll[(3)][(3)];
float P_pitch[(3)][(3)];

// State vector for roll and pitch
float x_roll[(3)];
float x_pitch[(3)];



// Keep track of time
float dt;
unsigned long time_prev;

// Noise Covariance
float Q[3];
float R[2];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print('a');      //handshake with matlab
  while (!Serial.available()); //wait for handshake to come back
  Serial.read();          //complete handshake
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
  Q[0] = 0.01;
  Q[1] = 0.003;
  R[0] = 0.003;
  R[1] = 0.003;

  // Initial time measurement
  time_prev = millis();

  // Set Kalman filter to verbose mode
  Kalman_verbose = false;
}

void loop() {
  // Get raw accelerometer and gyro data
  // TODO: Implement filters for data
  xAccel = (myIMU.accelRead(0) - xAccel_calib) / 1000;
  yAccel = (myIMU.accelRead(1) - yAccel_calib) / 1000;
  zAccel = (myIMU.accelRead(2) - zAccel_calib) / 1000;
  xGyro = (myIMU.gyroRead(0) - xGyro_calib) / RAD2DEG;
  yGyro = (myIMU.gyroRead(1) - yGyro_calib) / RAD2DEG;
  zGyro = (myIMU.gyroRead(2) - zGyro_calib) / RAD2DEG;

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
  //Serial.print("Accelerometer Pitch: "); 
  
  //  Serial.print("Roll: "); Serial.println(Roll * RAD2DEG);


  // Calculate using Kalman filter

  dt = ((float)millis() - (float)time_prev) / 1000;
  time_prev = millis();

  A[0][1] = dt;
  A[0][2] = -dt;
  MM.Transpose((float*) A, 3, 3, (float*) A_transpose);
  //  Serial.print("dt: ");  Serial.println(dt);
  kalman((float*)P_pitch, (float*)x_pitch, Pitch, xGyro);
  //MM.Print((float*) x_pitch, 3, 1, "pitch state vector");
  //Serial.print("Kalman Pitch ");

  Serial.print(millis()); Serial.print("\t");
  Serial.print(Pitch * RAD2DEG, 5);Serial.print("\t");
  Serial.println(x_pitch[0] * RAD2DEG, 5);
  //  Serial.print("Drift ");
  //  Serial.println(x_pitch[1] * RAD2DEG);

  //Serial.println( );
  delay(10);
}

void kalman(float *P, float *x, float angle_measured, float rate_measured) {
  // Set up variables for use
  float rate_estimated;
  float angle_estimated;
  float x_previous[3];
  float x_predicted[3];
  float P_predicted[3][3];
  //float P_old[3][3];

  MM.Copy(x, 3, 1, (float*) x_previous);
  //MM.Copy(P, 3, 3, (float*) P_old);

  // Step 2: Estimate a posteriori state estimate
  if (Kalman_verbose) {
    Serial.println("Step 2");
  }
  MM.Multiply((float*) A, (float*) x, 3, 3, 1, (float*) x_predicted);

  if (Kalman_verbose) {
    MM.Print((float*) x, 3, 1, "x");
    MM.Print((float*) x_predicted, 3, 1, "x_predicted");
  }
  // Step 3: Prediction error covariance
  if (Kalman_verbose) {
    Serial.println("Step 3");
  }
  {
    float multTemp[3][3];

    MM.Multiply((float*) A, (float*) P, 3, 3, 3, (float*) multTemp);

    if (Kalman_verbose) {
      MM.Print((float*) multTemp, 3, 3, "A P_old");
    }

    MM.Multiply((float*) multTemp, (float*) A_transpose, 3, 3, 3, (float*) P_predicted);
    if (Kalman_verbose) {
      MM.Print((float*) P_predicted, 3, 3, "P predicted");
    }
  }

  // Add process noise matrix
  P_predicted[0][0] += Q[0] ;
  P_predicted[1][1] += Q[1] ;
  P_predicted[2][2] += Q[2] ;

  if (Kalman_verbose) {
    MM.Print((float*) P_predicted, 3, 3, "P predicted + Q");
  }

  // Step 4: Kalman gain calculation

  if (Kalman_verbose) {
    Serial.println("Step 4");
  }
  float S[2][2];
  float K[3][2];
  {
    float temp1[2][3];
    MM.Multiply((float*) H, (float*) P_predicted, 2, 3, 3, (float*) temp1);
    if (Kalman_verbose) {
      MM.Print((float*) temp1, 2, 3, "H P predicted");
    }
    MM.Multiply((float*) temp1, (float*) H_T, 2, 3, 2, (float*) S);
    if (Kalman_verbose) {
      MM.Print((float*) S, 2, 2, "H P_predicted HT (S)");
    }
  }

  //MM.Print((float*) P_predicted, 3, 3, "P_predicted");

  S[0][0] += R[0];
  S[1][1] += R[1];

  if (Kalman_verbose) {
    MM.Print((float*) S, 2, 2, "S + R");
  }

  MM.Invert((float*) S, 2);

  if (Kalman_verbose) {
    MM.Print((float*) S, 2, 2, "S inverted");
  }
  {
    float temp[3][2];
    MM.Multiply((float*) P_predicted, (float*) H_T, 3, 3, 2, (float*) temp);
    if (Kalman_verbose) {
      MM.Print((float*) temp, 3, 2, "P_predicted H_T");
    }
    MM.Multiply((float*) temp, (float*) S, 3, 2, 2, (float*) K);
    if (Kalman_verbose) {
      MM.Print((float*) K, 3, 2, "K");
    }
  }


  // Step 5: New state vector calculation
  if (Kalman_verbose) {
    Serial.println("Step 5");
  }
  {
    float z[2];
    float residual[2];
    float temp[3];
    z[0] = angle_measured;
    z[1] = rate_measured;
    if (Kalman_verbose) {
      MM.Print((float*) z, 2, 1, "z measured");
    }
    float predicted_measurement[2];
    MM.Multiply((float*) H, (float*) x_predicted, 2, 3, 1, (float*) predicted_measurement);
    if (Kalman_verbose) {
      MM.Print((float*) predicted_measurement, 2, 1, "predicted_measurement");
    }
    MM.Subtract((float*) z, (float*) predicted_measurement, 2, 1, (float*) residual);
    if (Kalman_verbose) {
      MM.Print((float*) residual, 2, 1, "residual");
    }

    MM.Multiply((float*) K, (float*) residual, 3, 2, 1, (float*) temp);
    if (Kalman_verbose) {
      MM.Print((float*) temp, 3, 1, "K*residual");
    }
    MM.Add((float*) x_predicted, (float*) temp, 3, 1, (float*) x);
    if (Kalman_verbose) {
      MM.Print((float*) x, 3, 1, "x_predicted + K*residual");
    }
  }

  // Step 6: Update P Matrix
  if (Kalman_verbose) {
    Serial.println("Step 6");
  }
  {
    //    float P_old[3][3];
    //    MM.Copy((float*) P, 3, 3, (float*) P_old);

    float temp[3][3];
    MM.Multiply((float*) K, (float*) H, 3, 2, 3, (float*) temp);

    float temp2[3][3];
    MM.Multiply((float*) temp, (float*) P_predicted, 3, 3, 3, (float*) temp2);

    MM.Subtract((float*) P_predicted, (float*) temp2, 3, 3, (float*) P);
  }


  if (Kalman_verbose) {
    Serial.println("---------------");
  }
}

void calibrate () {
  //Serial.println("Beginning calibration routine...");
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

  //Serial.println("Calibration routine complete.");
}

