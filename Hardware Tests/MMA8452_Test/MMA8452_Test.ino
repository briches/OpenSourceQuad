// MMA8452 Accelerometer 
// by BR

// This sketch demonstrates interactions with the MMA8452 sensor using
// the library by N0M1 Design Ltd.
// 
// This sketch accesses and modifies all the registers in the 
// MMA8452 device (hopefully) and verifies they can be read.

#include <I2C.h>
#include <MMA8453_n0m1.h>
#include <PID_v1.h>


double Xin, Xout, Xsetpoint;
PID xPID(&Xin, &Xout, &Xsetpoint, 1, 1, 0, DIRECT);

MMA8453_n0m1 accel;

byte buf[42];




void setup() {
  
  Serial.begin(9600); 
  Serial.println("Begin Setup");
  accel.setI2CAddr(0x1D); // I2C address of the acceleromter
  Serial.println("I2C addr set")  ;
  accel.dataMode(true,2); //
  Serial.println("Data mode set");
  Serial.println("Setup Complete: Reading registers");
  Serial.println(" ");       
  
  buf[0] = REG_STATUS; //(R) Real time status
  buf[1] = REG_OUT_X_MSB; //(R) [7:0] are 8 MSBs of 10-bit sample
  buf[2] =  REG_OUT_X_LSB; //(R) [7:6] are 2 LSBs of 10-bit sample
  buf[3] =  REG_OUT_Y_MSB       ; //(R) [7:0] are 8 MSBs of 10-bit sample
  buf[4] =  REG_OUT_Y_LSB; //(R) [7:6] are 2 LSBs of 10-bit sample
  buf[5] =  REG_OUT_Z_MSB; //(R) [7:0] are 8 MSBs of 10-bit sample
  buf[6] =  REG_OUT_Z_LSB; //(R) [7:6] are 2 LSBs of 10-bit sample
  buf[7] =  REG_SYSMOD; //(R) Current system mode
  buf[8] =  REG_INT_SOURCE; //(R) Interrupt status
  buf[9] =  REG_WHO_AM_I; //(R) Device ID (0x3A)
  buf[10] =  REG_XYZ_DATA_CFG; //(R/W) Dynamic range settings
  buf[11] =  REG_HP_FILTER_CUTOFF; //(R/W) cut-off frequency is set to 16Hz @ 800Hz
  buf[12] =  REG_PL_STATUS; //(R) Landscape/Portrait orientation status
  buf[13] =  REG_PL_CFG; //(R/W) Landscape/Portrait configuration
  buf[14] =  REG_PL_COUNT; //(R) Landscape/Portrait debounce counter
  buf[15] =  REG_PL_BF_ZCOMP; //(R) Back-Front, Z-Lock trip threshold
  buf[16] =  REG_P_L_THS_REG; //(R/W) Portrait to Landscape trip angle is 29 degree
  buf[17] =  REG_FF_MT_CFG; //(R/W) Freefall/motion functional block configuration
  buf[18] =  REG_FF_MT_SRC; //(R) Freefall/motion event source register
  buf[19] =  REG_FF_MT_THS; //(R/W) Freefall/motion threshold register = 
  buf[20] =  REG_FF_MT_COUNT; //(R/W) Freefall/motion debounce counter
  buf[21] =  REG_TRANSIENT_CFG; //(R/W) Transient functional block configuration
  buf[22] =  REG_TRANSIENT_SRC; //(R) Transient event status register
  buf[23] =  REG_TRANSIENT_THS; //(R/W) Transient event threshold
  buf[24] =  REG_TRANSIENT_COUNT; //(R/W) Transient debounce counter
  buf[25] =  REG_PULSE_CFG; //(R/W) ELE, Double_XYZ or Single_XYZ
  buf[26] =  REG_PULSE_SRC; //(R) EA, Double_XYZ or Single_XYZ
  buf[27] =  REG_PULSE_THSX; //(R/W) X pulse threshold
  buf[28] =  REG_PULSE_THSY; //(R/W) Y pulse threshold
  buf[29] =  REG_PULSE_THSZ; //(R/W) Z pulse threshold
  buf[30] =  REG_PULSE_TMLT; //(R/W) Time limit for pulse
  buf[31] =  REG_PULSE_LTCY; //(R/W) Latency time for 2nd pulse
  buf[32] =  REG_PULSE_WIND; //(R/W) Window time for 2nd pulse
  buf[33] =  REG_ASLP_COUNT; //(R/W) Counter setting for auto-sleep
  buf[34] =  REG_CTRL_REG1; //(R/W) ODR = 800 Hz, STANDBY mode
  buf[35] =  REG_CTRL_REG2; //(R/W) Sleep enable, OS Modes, RST, ST
  buf[36] =  REG_CTRL_REG3; //(R/W) Wake from sleep, IPOL, PP_OD
  buf[37] =  REG_CTRL_REG4; //(R/W) Interrupt enable register
  buf[38] =  REG_CTRL_REG5; //(R/W) Interrupt pin (INT1/INT2) map
  buf[39] =  REG_OFF_X; //(R/W) X-axis offset adjust
  buf[40] =  REG_OFF_Y; //(R/W) Y-axis offset adjust
  buf[41] =  REG_OFF_Z; //(R/W) Z-axis offset adjust
  
  
  Serial.println("**********************************");
  regPrint(buf);  
  Serial.println("**********************************");
  
  
  Xsetpoint = 100;
  xPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-500,500);
  xPID.SetSampleTime(1);
  
}


void loop () {
  
  accel.update();
  int x = accel.x();
  Serial.print(x);
  Xin = x;
  xPID.Compute();
  Serial.print(" PID: ");
  Serial.println(Xout);
}


/********************************************/
//      regPrint(byte buf[])
/********************************************/

  void regPrint(byte buf[]) {
    byte readValue;
    byte readReg;
    
    for(int i = 0; i <= 41; i++) {
      readReg = buf[i];
      accel.regRead(readReg,&readValue,1);
      
      Serial.print("Register: 0x"); Serial.print(buf[i], HEX); 
      Serial.print("  ");
      Serial.print("Binary value: ");Serial.print(readValue,BIN); 
      Serial.print("  ");
      Serial.print("Hex Value: "); Serial.println(readValue,HEX);
    }
  }
