#include <Arduino.h>
#include "osd.h"

uint8_t first_half;   // >>>> >>>> 0001 0110
uint8_t sencond_half; // ____ ____ 0100 0111
uint16_t osd_buffer;

void osd_display(int frame, float phi, float theta, float psi)
{
  // Frame 1: [B10000001B10000001,(theta+90)*200, (phi+180)*50, (psi)*50]
  // Frame 2: [B10000010B10000010, motor_armed, throttle, control mode]
  // Frame 3: [B10000011B10000011, altitude, hdot, ultrasonic altitude]
  // Frame 4: [B10000100B10000100, RSSI, batt volt, batt A]
  // Frame 5: [B10000101B10000101, dist home, dir home, spare]
  // Frame 6: [B10000110B10000110, GPS Lat, GPS Lon (3 bytes each)] 
  // Frame 7: [B10000111B10000111, GPS lock, GPS satellites, spare]  
  // Frame 8: [B10001000B10001000, spare, spare, spare] 
  // Frame 9: [B10001001B10001001, spare, spare, spare] 
  // Frame10: [B10001010B10001010, spare, spare, spare] 
  
   if(frame == 1)
   {
    Serial.write(B11111110);
    Serial.write(B11111110);
    osd_buffer = (short int)((theta+90)*200);
    first_half   = osd_buffer >> 8;   // >>>> >>>> 0001 0110
    sencond_half = osd_buffer & B11111111; // ____ ____ 0100 0111
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)((phi+180)*50);
    first_half   = osd_buffer >> 8;   // >>>> >>>> 0001 0110
    sencond_half = osd_buffer & B11111111; // ____ ____ 0100 0111
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)((psi)*50);
    first_half   = osd_buffer >> 8;   // >>>> >>>> 0001 0110
    sencond_half = osd_buffer & B11111111; // ____ ____ 0100 0111
    Serial.write(first_half);
    Serial.write(sencond_half);
   }
  
  
}
