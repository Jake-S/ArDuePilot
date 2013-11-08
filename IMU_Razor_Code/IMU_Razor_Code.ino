#include <Wire.h>
#include "sensors.h"

byte a_bytes[6];
byte m_bytes[6];
byte w_bytes[6];

void setup() {
  Wire.begin();
  Serial.begin(57600);

  i2c_write(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, ADXL_PWRCTL_MEASURE);
  i2c_write(HMC5843_ADDRESS, HMC5843_REGISTER_MEASMODE, HMC5843_MEASMODE_CONT);
  i2c_write(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, ITG3200_FULLSCALE | ITG3200_42HZ);
}

void loop() {
  
  /* To Do:
  add a serial interrupt:
  constantly query i2c
  turn off interrupts and update stored vars
  if ever an interrupt on UART, send stored vars (near instanteous reply)
  */

  i2c_read(ADXL345_ADDRESS, ADXL345_REGISTER_XLSB, 6, a_bytes);
  if(Serial.available()>0)
  {
    for(int j=0;j<6;j++) Serial.write(a_bytes[j]);
    for(int j=0;j<6;j++) Serial.write(m_bytes[j]);
    for(int j=0;j<6;j++) Serial.write(w_bytes[j]);
    while(Serial.available()>0) Serial.read();
  }
 i2c_read(HMC5843_ADDRESS, HMC5843_REGISTER_XMSB, 6, m_bytes);
  if(Serial.available()>0)
  {
    for(int j=0;j<6;j++) Serial.write(a_bytes[j]);
    for(int j=0;j<6;j++) Serial.write(m_bytes[j]);
    for(int j=0;j<6;j++) Serial.write(w_bytes[j]);
    while(Serial.available()>0) Serial.read();
  }
 i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_XMSB, 6, w_bytes);
  if(Serial.available()>0)
  {
    for(int j=0;j<6;j++) Serial.write(a_bytes[j]);
    for(int j=0;j<6;j++) Serial.write(m_bytes[j]);
    for(int j=0;j<6;j++) Serial.write(w_bytes[j]);
    while(Serial.available()>0) Serial.read();
  }


}

