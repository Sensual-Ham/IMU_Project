// Harrison King
// 2/10/2017
// IMU.c: the actual function definitions for things

#include "Arduino.h"
#include "IMU.h"

IMU::IMU(){
  Wire.begin();
  accelWrite(PWR_CNTRL,WAKE); //power the damn thing on
};

float IMU::accelRead(int axisnum){
  Wire.beginTransmission(ACC_ADD);
  Wire.write(ACC_DATA+axisnum*2); //set to read from x, y, or z registers
  Wire.endTransmission();
  
  Wire.beginTransmission(ACC_ADD);
  Wire.requestFrom(ACC_ADD,2);
  int input = ((Wire.read())|(Wire.read()<<8)); //get data
  Wire.endTransmission();
  float out = input*accfact; //correct for scaling factor
  return out; //spit out data
}

float IMU::gyroRead(int axisnum){
  Wire.beginTransmission(GYR_ADD);
  Wire.write(GYR_DATA+axisnum*2); //set to read from x, y, or z registers
  Wire.endTransmission();
  
  Wire.beginTransmission(GYR_ADD);
  Wire.requestFrom(GYR_ADD,2);
  int input = ((Wire.read()<<8)|(Wire.read())); //get data
  Wire.endTransmission();
  float out = input/gyrofact; //correct for scaling factor
  return out; //spit out data
}

void IMU::accelWrite(int reg, int val){
  Wire.beginTransmission(ACC_ADD);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void IMU::gyroWrite(int reg, int val){
  Wire.beginTransmission(GYR_ADD);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
