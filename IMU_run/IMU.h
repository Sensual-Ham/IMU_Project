// Harrison King
// 2/10/2017
// IMU.h: a library for AERO 465 project 1

#ifndef IMU_h
#define IMU_h

#include "Arduino.h"
#include "Wire.h"


#define ACC_ADD 0x53
#define PWR_CNTRL 0x2D
#define SLEEP 0x00
#define WAKE 0b00001000
#define ACC_DATA 0x32
#define FORMAT 0x31
#define DEV_ID 0x00

#define GYR_ADD 0b1101000 //or 1101000
#define GYR_DATA 0x1D

class IMU{
	public:
	  IMU();
	  float accelRead(int axisnum);
	  float gyroRead(int axisnum);
	  void accelWrite(int reg, int val);
	  void gyroWrite(int reg, int val);
	  void init();
	private:
	  float accfact = 3.9;
	  float gyrofact = 14.375;
};


#endif
