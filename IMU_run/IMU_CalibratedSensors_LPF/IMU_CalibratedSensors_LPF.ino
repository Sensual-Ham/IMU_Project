
#include <Wire.h>


// Accelerometer Global Variables

#define accADD 0x53 
//char PowerReg = 0x2D;
//char measureMode = 0x08;
//char gRangeReg = 0x31;
//char g_range = 0x00; // +/- 2g
byte sensorMeas[6]; // Sensor Accelrometer Measurements 
byte numByte = 6;
int accConversion = 256;

char DataX0 = 0x32;

float axRaw;
float ayRaw;
float azRaw;
float axCal;
float ayCal;
float azCal;
float axReal;
float ayReal;
float azReal;


//Rate Gyro Variables

#define gyroAdd 0x68
byte gyroMeas[6]; // Sensor Gyro Measurements 
//char simple_div = 0x15;
//char DLPF_FS_reg = 0x16;
//char DLPF_FS_SEL_0 = 1 << 3;
//char DLPF_FS_SEL_1 = 1 << 4;
//char DLPF_CFG_0 = 1;
//char INT_CFG = 0x17;
//char PWR_MGM = 0x3E;
char GYRO_XOUT_H = 0x1D;
char gyroConversion = 14.375;

float gxRaw;
float gyRaw;
float gzRaw;
float gxCal;
float gyCal;
float gzCal;
float gxReal;
float gyReal;
float gzReal;


//Low Pass filter varaibles

int alpha = 0.90; //Smaller alpha --> Lower Noise, Larger alpha --> Higher Noise
float axFilter;
float ayFilter;
float azFilter;
float axFilterHold;
float ayFilterHold;
float azFilterHold;

float gxFilterHold; 
float gyFilterHold; 
float gzFilterHold;
float gxFilter;
float gyFilter;
float gzFilter;


void setup(){
  
// Setup Accelerometer

Wire.begin();
Serial.begin(9600);


//Set up the accelerometer

//SendtoDevice(acc_add,Power_reg, Measure_mode);
////Set up the accelerometer to the write range
//SendtoDevice(acc_add,g_range_reg, g_range);
//
////Setting up rategyro
//
//SendtoDevice(gyro_add,DLPF_FS_reg,(DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0));
//SendtoDevice(gyro_add,simple_div,9);
//SendtoDevice(gyro_add,INT_CFG,0x00);
//SendtoDevice(gyro_add,PWR_MGM,0x00);

calibrateData();

 Serial.print("Accelerometer Raw Data (G)");
 Serial.print("\t ");
 Serial.print("Accelerometer Calibrated Data (G)");
 Serial.print("\t ");
 
 Serial.print("Rate Gyro Raw Data (G)");
 Serial.print("\t    ");
 Serial.print("Rate Gyro Calibrated Data (G)");
 Serial.print("\t ");


 
}

void loop(){
 
  //Raw Acceleration Data
  axRaw = ((sensorMeas[1] << 8) | sensorMeas[0])/accConversion;
  ayRaw = ((sensorMeas[3] << 8) | sensorMeas[2])/accConversion;
  azRaw = ((sensorMeas[5] << 8) | sensorMeas[4])/accConversion;
  
  getData(accADD,DataX0,sensorMeas,numByte);
  
  
  //Calibrated Acceleration Data
  axReal = axRaw - axCal;
  ayReal = ayRaw - ayCal;
  azReal = azRaw + azCal;
  
  //Raw Data (Accelerometer)
  Serial.print(axRaw);
  Serial.print("\t ");
  Serial.print(ayRaw);
  Serial.print("\t ");
  Serial.print(azRaw);
  Serial.print("\t    ");
  Serial.print("\t  ");


  //Calibrated Data (Accelerometer)
  Serial.print(axReal);
  Serial.print("\t");
  Serial.print(ayReal);
  Serial.print("\t");
  Serial.print(azReal);
  Serial.print("\t    ");
  Serial.print("\t       ");
 
  gxRaw = ((gyroMeas[0] << 8 | gyroMeas[1]))/gyroConversion;
  gyRaw = ((gyroMeas[2] << 8 | gyroMeas[3]))/gyroConversion;
  gzRaw = ((gyroMeas[4] << 8 | gyroMeas[5]))/gyroConversion;
  
  getData(gyroAdd,GYRO_XOUT_H,gyroMeas,numByte);
  
  gxReal = gxRaw + gxCal;
  gyReal = gyRaw + gyCal;
  gzReal = gzRaw + gzCal;
  
  
  //Raw Data (Rate Gyro)
  Serial.print(gxRaw);
  Serial.print("\t");
  Serial.print(gyRaw);
  Serial.print("\t");
  Serial.print(gzRaw);
  Serial.print("\t    ");
  Serial.print("\t  ");
 

  //Calibrated Data (Rate Gyro)
  Serial.print(gxReal);
  Serial.print("\t  ");
  Serial.print(gyReal);
  Serial.print("\t");
  Serial.print(gzReal);
  Serial.println(" ");

// Low Pass Filter
axFilter = alpha*axReal + (1-alpha)*axFilterHold;
ayFilter = alpha*ayReal + (1-alpha)*ayFilterHold;
azFilter = alpha*azReal + (1-alpha)*azFilterHold;

axFilterHold = axFilter;
ayFilterHold = ayFilter;
azFilterHold = azFilter;

gxFilter = alpha*gxReal + (1-alpha)*gxFilterHold;
gyFilter = alpha*gyReal + (1-alpha)*gyFilterHold;
gzFilter = alpha*gzReal + (1-alpha)*gzFilterHold;

gxFilterHold = gxFilter;
gyFilterHold = gyFilter;
gzFilterHold = gzFilter;
  
}

//------------------ functions --------------------------------
void SendtoDevice(char deviceAdd, char regAdd, char Opr)
{
  Wire.beginTransmission(deviceAdd); 
  Wire.write(regAdd);
  Wire.write(Opr);
  Wire.endTransmission();
}

void getData(char sensorAdd, char sensorReg, byte sensorMeas[], int numByte)
{

  Wire.beginTransmission(sensorAdd);
  Wire.write(sensorReg);
  Wire.endTransmission();
  Wire.beginTransmission(sensorAdd);
  Wire.requestFrom(sensorAdd,numByte);
  
  int k = 0;
  while(Wire.available()){
    sensorMeas[k] = Wire.read();
    k++;
  }
}

void calibrateData(void){
  
int sampleSize = 2000;
int counter = 0;
do{
  //accelerometer calibration
  axRaw = ((sensorMeas[1] << 8) | sensorMeas[0])/accConversion;
  ayRaw = ((sensorMeas[3] << 8) | sensorMeas[2])/accConversion;
  azRaw = ((sensorMeas[5] << 8) | sensorMeas[4])/accConversion;
  
  getData(accADD,DataX0,sensorMeas,numByte);
  
  //subtracting out the inital data
  axCal = axCal - axRaw;
  ayCal = ayCal - ayRaw;
  azCal = azCal - azRaw;
  
  counter++;
} while (counter != sampleSize);
  
  axCal = axCal/sampleSize;
  ayCal = ayCal/sampleSize;
  azCal = azCal/sampleSize;
  
  //rate gyro calibration
int counter2 = 0;
do{
  
  //getData(gyro_add,GYRO_XOUT_H,gyro_mea,numbyte);
  gxRaw = ((gyroMeas[0] << 8 | gyroMeas[1]))/gyroConversion;
  gyRaw = ((gyroMeas[2] << 8 | gyroMeas[3]))/gyroConversion;
  gzRaw = ((gyroMeas[4] << 8 | gyroMeas[5]))/gyroConversion;
  
  getData(gyroAdd,GYRO_XOUT_H,gyroMeas,numByte);
  
  gxCal = gxCal - gxRaw;
  gyCal = gyCal - gyRaw;
  gzCal = gzCal - gzRaw;
  
  counter2++;
}while(counter2 != sampleSize);
  
  gxCal = gxCal/sampleSize;
  gyCal = gyCal/sampleSize;
  gzCal = gzCal/sampleSize;
  
}
