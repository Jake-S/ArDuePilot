#include <Arduino.h>
#include "osd.h"

uint8_t first_half;  
uint8_t sencond_half; 

uint16_t osd_buffer;
uint16_t spare = 0;

// Package data and send to OSD/Telemetry
void osd_display(int frame, float phi, float theta, float psi, int dt, int motor_armed, float throttle, int control_mode, float altitude, float hdot, float ultra_altitude, float rssi, float batt_v, float batt_a, float dist_home, float dir_home, float lat, float lon, int gps_lock, int gps_sats)
{
  // Frame 1: [B10000001B10000001,(theta+90)*200, (phi+180)*50, (psi)*50]
  // Frame 2: [B10000010B10000010, dt, motor_armed, throttle, control mode]
  // Frame 3: [B10000011B10000011, altitude, hdot, ultrasonic altitude]
  // Frame 4: [B10000100B10000100, RSSI, batt volt, batt A]
  // Frame 5: [B10000101B10000101, dist home, dir home, spare]
  // Frame 6: [B10000110B10000110, GPS Lat (4 bytes), GPS lock] 
  // Frame 7: [B10000111B10000111, GPS lon (4 bytes), GPS satellites]  
  // Frame 8: [B10001000B10001000, spare, spare, spare] 
  // Frame 9: [B10001001B10001001, spare, spare, spare] 
  // Frame10: [B10001010B10001010, spare, spare, spare] 

  
   if(frame == 1)
   {
    Serial.write(B10000001);
    Serial.write(B10000001);
    osd_buffer = (short int)((theta+90)*80);
    first_half   = osd_buffer >> 8;   // >>>> >>>> ########
    sencond_half = osd_buffer & B11111111; // ######## ________
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)((phi+180)*40);
    first_half   = osd_buffer >> 8;  
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)((psi)*40);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
   }
   else if(frame == 2)
  {
    Serial.write(B10000010);
    Serial.write(B10000010);
    Serial.write((uint8_t)(dt));
    Serial.write((uint8_t)(motor_armed));
    osd_buffer = (short int)(throttle*150);
    first_half   = osd_buffer >> 8;  
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(control_mode);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
  }
    else if(frame == 3)
  {
    Serial.write(B10000011);
    Serial.write(B10000011);
    osd_buffer = (short int)(altitude);
    first_half   = osd_buffer >> 8;   // >>>> >>>> ########
    sencond_half = osd_buffer & B11111111; // ######## ________
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(hdot*100);
    first_half   = osd_buffer >> 8;  
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(ultra_altitude*200);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
  }
  else if(frame == 4)
  {
    Serial.write(B10000100);
    Serial.write(B10000100);
    osd_buffer = (short int)(rssi);
    first_half   = osd_buffer >> 8;   // >>>> >>>> ########
    sencond_half = osd_buffer & B11111111; // ######## ________
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(batt_v*100);
    first_half   = osd_buffer >> 8;  
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(batt_a*100);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
  }
    else if(frame == 5)
  {
    Serial.write(B10000101);
    Serial.write(B10000101);
    osd_buffer = (short int)(dist_home);
    first_half   = osd_buffer >> 8;   // >>>> >>>> ########
    sencond_half = osd_buffer & B11111111; // ######## ________
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(dir_home*40);
    first_half   = osd_buffer >> 8;  
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(spare);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
  }
   else if(frame == 6)
  {
    Serial.write(B10000110);
    Serial.write(B10000110);
    osd_buffer = (short int)(lat*100);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)((lat*100 - (float)(osd_buffer))*1000);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(gps_lock);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
  }
  else if(frame == 7)
  {
    Serial.write(B10000111);
    Serial.write(B10000111);
    osd_buffer = (short int)(lon*100);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)((lon*100 - (float)(osd_buffer))*1000);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(gps_sats);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
  }
}
