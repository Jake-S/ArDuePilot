// Arduino MAVLink test code.

#include "osd.h"

float phi = 0;
float theta = 0;
float psi = 0;

int frame = 0;

       
void setup() {
        Serial.begin(57600);
  
        delay(500);
        
       phi = 15;
       theta  = 30;
       psi = 90;
}

void loop() {
  
    frame += frame;
    if(frame > 10) frame = 1;

    psi = psi + .5;
    
    
   
  frame = 1;
  osd_display(frame, phi, theta, psi);
   
     delay(500);
     
}
