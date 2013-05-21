// OSEPP Gyroscope Sensor Test Sketch
// by BR

// This sketch demonstrates interactions with the Gyroscope Sensor using OseppGyro.h
// Uses the library I wrote (format copied from the MMA library)


#include <I2C.h>
#include <OseppGyro.h>

OseppGyro gyro;

byte buf[35]; // Stores the register values

int ScaleRange = 2000;
int DLPF = 6;



void setup()
{
 delay(1000);
 // Start the serial port for output
 Serial.begin(9600);
 gyro.setI2CAddr(0x69);
 gyro.dataMode(ScaleRange,DLPF); 
 
 
 Serial.println("setup");
  buf[0] = WHO_AM_I;
  buf[1] = PRODUCT_ID;
  buf[2] =  X_OFFS_USRH;// (R/W) User offset of H byte of X gyro (2's complement)
  buf[3] =  X_OFFS_USRL; // (R/W) User offset of L byte of X gyro (2's complement)
  buf[4] =  Y_OFFS_USRH; // (R/W) User offset of H byte of Y gyro (2's complement)
  buf[5] =  Y_OFFS_USRL; // (R/W) User offset of L byte of Y gyro (2's complement)
  buf[6] =  Z_OFFS_USRH; // (R/W) User offset of H byte of Z gyro (2's complement)
  buf[7] =  Z_OFFS_USRL; // (R/W) User offset of L byte of Z gyro (2's complement)
  buf[8] =  FIFO_EN;
  buf[9] =  AUX_VDDIO;
  buf[10] = AUX_SLV_ADDR;
  buf[11] = SMPLRT_DIV; // (R/W) Sample rate divider, divides analog sample rate
  buf[12] = DLPF_FS_SYNC; // {R/W) EXT_SYNC_SET     FS_SEL      DLPF_CFG || Default is 0
  buf[13] = INT_CFG; // (R/W) Configures interrupt operation
  buf[14] = AUX_ADDR;
  buf[15] = INT_STATUS; // (R) Interrupt status
  buf[16] = TEMP_OUT_H;
  buf[17] = TEMP_OUT_L;
  buf[18] = GYRO_XOUT_H; // (R) 16 bit x gyro data (2's complement)
  buf[19] = GYRO_XOUT_L; 
  buf[20] = GYRO_YOUT_H; // (R) 16 bit y gyro data (2's complement)
  buf[21] = GYRO_YOUT_L;
  buf[22] = GYRO_ZOUT_H; // (R) 16 bit z gyro data (2's complement)
  buf[23] = GYRO_ZOUT_L;
  buf[24] = AUX_XOUT_H;
  buf[25] = AUX_XOUT_L;
  buf[26] = AUX_YOUT_H;
  buf[27] = AUX_YOUT_L;
  buf[28] = AUX_ZOUT_H;
  buf[29] = AUX_ZOUT_L;
  buf[30] = FIFO_COUNTH;
  buf[31] = FIFO_COUNTL;
  buf[32] = FIFO_R;
  buf[33] = USER_CTRL; // (R/W) Enable/Disable various modes on the sensor
  buf[34] = PWR_MGM; // (R/W) manage the power control, select clock source ( can be internal oscillator, external oscillator, x,y,z gyro oscillator)  

  
  Serial.println("*************************************************");
  Serial.println("***********Edit Register Values******************");
  //regEdit(buf);
  Serial.println("**********/Edit Register Values******************");
  Serial.println("*************************************************");

  Serial.println(" ");

  Serial.println("*************************************************");
  Serial.println("****************Register Values******************");
  regPrint(buf);  
  Serial.println("****************/Register Values*****************");
  Serial.println("*************************************************");

  
}

// Main program loop
void loop()
{

  gyro.update();
  Serial.print("X: "); Serial.print(gyro.x());
  Serial.print(" Y: "); Serial.print(gyro.y());
  Serial.print(" Z: "); Serial.println(gyro.z());


}

/********************************************/
//           regPrint(byte buf[])           //
/********************************************/

void regPrint(byte buf[]) {
  byte readValue;
  byte readReg;
  
  for(int i = 0; i <= 35; i++) {
    readReg = buf[i];
    gyro.regRead(readReg,&readValue,1);
    
    Serial.print("Register: 0x"); Serial.print(buf[i], HEX); 
    Serial.print("  ");
    Serial.print("Binary value: ");Serial.print(readValue,BIN); 
    Serial.print("  ");
    Serial.print("Hex Value: "); Serial.println(readValue,HEX);
  }
}

