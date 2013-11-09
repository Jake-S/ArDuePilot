
#include "osd.h"

float phi = 0;
float theta = 0;
float psi = 0;

int frame = 0;
int dt = 0;
int motor_armed = 1;
float throttle = 59.21;
int control_mode = 1;
       
     
 int gps_lock = 3;
 int gps_sats = 9;
   
float altitude = 500;
float hdot = -10;
float ultra_altitude = 500;

float rssi = 255;
float batt_v = 19.99;
float batt_a = 99.99;
float dist_home = 0;
float dir_home = 0;
float lat = 32.12345678;
float lon = -118.12345678;

void setup() {
        Serial.begin(57600);
  
        delay(500);
        
       phi = 15;
       theta  = 30;
       psi = 90;
}

void loop() {
  
    frame++;
    if(frame > 10) frame = 1;

    psi = psi + .02;
    
    dt = 50;
   
   //frame = 1;
   osd_display(frame, phi, theta, psi, dt, motor_armed, throttle, control_mode, altitude, hdot, ultra_altitude, rssi, batt_v, batt_a, dist_home, dir_home, lat, lon, gps_lock, gps_sats);

 
     delay(5);
     
}
