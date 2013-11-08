#include <Wire.h>
#include "sensors.h"

void i2c_read(int address, byte reg, int count, byte* data) {
  // Send input register address
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  // Connect to device and request bytes
  Wire.beginTransmission(address);
  Wire.requestFrom(address,count);
  int i = 0;
  while(Wire.available()) // slave may send less than requested
  {
    char c = Wire.read(); // receive a byte as character
    data[i] = c;
    i++;
  }
  Wire.endTransmission();
}

void i2c_write(int address, byte reg, byte data) {
  // Send output register address
  Wire.beginTransmission(address);
  Wire.write(reg);
  // Connect to device and send byte
  Wire.write(data); // low byte
  Wire.endTransmission();
}



