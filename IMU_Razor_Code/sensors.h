#ifndef __SENSORS_H
#define __SENSORS_H

// I2C Vars:
// ACCEL
#define  ADXL345_ADDRESS (0xA6 >> 1)
#define ADXL345_REGISTER_XLSB (0x32) // LSB register
#define ADXL_REGISTER_PWRCTL (0x2D) //Need to set power control bit to wake up the adxl345
#define ADXL_PWRCTL_MEASURE (1 << 3)
//Magnetometer
#define HMC5843_ADDRESS (0x3C >> 1)
 #define HMC5843_REGISTER_XMSB (0x03) //First data address of 6 is XMSB.  Also need to set a configuration register for continuous measurement
 #define HMC5843_REGISTER_MEASMODE (0x02)
 #define HMC5843_MEASMODE_CONT (0x00)
// GYRO
#define ITG3200_ADDRESS (0xD0 >> 1)
#define ITG3200_REGISTER_XMSB (0x1D) //request burst of 6 bytes from this address
#define ITG3200_REGISTER_DLPF_FS (0x16)
#define ITG3200_FULLSCALE (0x03 << 3)
#define ITG3200_42HZ (0x03)

#include "Arduino.h"
//i2c comm
void i2c_read(int address, byte reg, int count, byte* data);
void i2c_write(int address, byte reg, byte data);

#endif //__SENSORS_H
