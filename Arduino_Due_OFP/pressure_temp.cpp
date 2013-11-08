#include <Wire.h>
#include "pressure_temp.h"
#include <Arduino.h>

void bmp085Calibration(short int *ac1_ptc,short int *ac2_ptc,short int *ac3_ptc,unsigned int *ac4_ptc,unsigned int *ac5_ptc,unsigned int *ac6_ptc,short int *b1_ptc,short int *b2_ptc,short int *mb_ptc,short int *mc_ptc,short int *md_ptc)
{
  *ac1_ptc = bmp085ReadInt(0xAA);
  *ac2_ptc = bmp085ReadInt(0xAC);
  *ac3_ptc = bmp085ReadInt(0xAE);
  *ac4_ptc = bmp085ReadInt(0xB0);
  *ac5_ptc = bmp085ReadInt(0xB2);
  *ac6_ptc = bmp085ReadInt(0xB4);
  *b1_ptc = bmp085ReadInt(0xB6);
  *b2_ptc = bmp085ReadInt(0xB8);
  *mb_ptc = bmp085ReadInt(0xBA);
  *mc_ptc = bmp085ReadInt(0xBC);
  *md_ptc = bmp085ReadInt(0xBE);
}

int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2);
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

short bmp085GetTemperature(unsigned int ut, unsigned int ac5_ptc,unsigned int ac6_ptc, short int mc_ptc,short int md_ptc, long *b5_ptc)
{
  long x1, x2;
  float temp;

  x1 = (((long)ut - (long)ac6_ptc)*(long)ac5_ptc) >> 15;
  x2 = ((long)mc_ptc <<11)/(x1 + md_ptc);
  
  temp =((x1 + x2 + 8)>>4)*.1*9.0/5.0 + 32.0; // Deg F
  
  *b5_ptc = x1 + x2; 
  return (temp);  
}


// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}




long bmp085GetPressure(unsigned long up,short int ac1_ptc,short int ac2_ptc,short int ac3_ptc,unsigned int ac4_ptc,short int b1_ptc,short int b2_ptc,long b5_ptc,const unsigned char OSS)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5_ptc - 4000;
  // Calculate B3
  x1 = (b2_ptc * (b6 * b6)>>12)>>11;
  x2 = (ac2_ptc * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1_ptc)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3_ptc * b6)>>13;
  x2 = (b1_ptc * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4_ptc * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p; // in Pa
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}



// Read the uncompensated pressure value
unsigned long bmp085ReadUP(const unsigned char OSS)
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}

