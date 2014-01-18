
#include <Arduino.h>
#include "calibration.h" //for B_DECLINATION

#define pi 3.14159265359

// WARNING WARNING WARNING
// THIS REQUIRES A SERIAL BUFFER OF SIZE 66 OR GREATER, MUST EDIT RINGBUFFER.H library!!!!
byte gps_msg[67];
int msg_lng = 66;
byte send_msg[22];

int getGPSdata(int *gps_fix, int *num_sats, double *lat, double *lon, double *gps_alt_m)
{

  int gps_data = 0; // = 0 = no data, 1 = data, 2 = checksum error
  int gps_string_valid = 1;


  while(Serial2.available() >= msg_lng)
  {
    if(gps_string_valid == 1)
    {
      gps_msg[0] = Serial2.read();
      gps_msg[1] = Serial2.read();
      gps_msg[2] = Serial2.read();
      gps_msg[3] = Serial2.read();
      gps_msg[4] = Serial2.read();
    }
    else
    {
      for(int i=0; i<4; i++)
      {
        gps_msg[i] = gps_msg[i+1];
      }
      gps_msg[4] = Serial2.read();
    }

    if(gps_msg[0]==0xA0 && gps_msg[1]==0xA1 && gps_msg[2]==0x00 && gps_msg[3]==0x3B && gps_msg[4]==0xA8)
    {
      // Data message
      for(int i=5;i<=msg_lng-1; i++)
      {
        gps_msg[i] = Serial2.read();
      }
      

     // Check checksum
      byte CS = 0x00;
      for(int i = 4;i<msg_lng-4;i++)
      {
        CS = CS^gps_msg[i];
      }
      byte checksum = gps_msg[msg_lng-3];


      if (CS == checksum)
      {
        // Data good, store into variables
        gps_data = 1;

        *gps_fix = (int)gps_msg[3+2];
        *num_sats= (int)gps_msg[3+3];
        

         *lat = (double)( (int)gps_msg[3+10] * 16777216 + (int)gps_msg[3+11]*65536 + (int)gps_msg[3+12]*256 + (int)gps_msg[3+13]) / 10000000;
         *lon = (double)( (int)gps_msg[3+14] * 16777216 + (int)gps_msg[3+15]*65536 + (int)gps_msg[3+16]*256 + (int)gps_msg[3+17]) / 10000000;

        *gps_alt_m = (double)( (int)gps_msg[3+22] * 16777216 + (int)gps_msg[3+23]*65536 + (int)gps_msg[3+24]*256 + (int)gps_msg[3+25]) / 100;

      }
      else
      {
        gps_data = 2;
      }

      while(Serial2.available() >0)
      {
        Serial2.read();
        }

    }
    else{
      gps_string_valid = 0;
    }


  }




  return(gps_data);
}


void GPSwarmBoot(float lat, float lon, int altitude)
{
  
  byte lat_array[2];
  byte lon_array[2];
  byte altitude_array[2];
   
  int lat_int = int(lat*100);
  int lon_int = int(lon*100);
  
  
   
  lat_array[0] = lat_int;
  lat_array[1] = lat_int >> 8;
  
  lon_array[0] = lon_int;
  lon_array[1] = lon_int >> 8;
  
  altitude_array[0] = altitude;
  altitude_array[1] = altitude >> 8;
  
  // Start
  send_msg[0] = 0xA0;
  send_msg[1] = 0xA1;
  //Payload
  send_msg[2] = 0x00;
  send_msg[3] = 0x0F;
  // MSG ID
  send_msg[4] = 0x01; //1
  // MSG
  send_msg[5] = 0x02;//2
  send_msg[6] = 0x07;//3 year
  send_msg[7] = 0xDE;//4 Year
  send_msg[8] = 0x01;//5 Month
  send_msg[9] = 0x01;//6 Day
  send_msg[10] = 0x01;//7 Hour
  send_msg[11] = 0x01;//8 Minute
  send_msg[12] = 0x01;//9 Second
  send_msg[13] = lat_array[1]; //10 Lat
  send_msg[14] = lat_array[0]; //11 lat 
  send_msg[15] = lon_array[1]; //12 lon
  send_msg[16] = lon_array[0]; //13 lon
  send_msg[17] = altitude_array[1]; //14 altitude
  send_msg[18] = altitude_array[0]; //15 altitude
  // CS
  byte CS = 0x00;
      for(int i = 0;i<=14;i++)
      {
        CS = CS^send_msg[i+4];
      }
  send_msg[19] = CS;
  // END
  send_msg[20] = 0x0D;
  send_msg[21] = 0x0A;
  
 
    Serial2.write(send_msg,22);
    delay(2000);
    while(Serial2.available()>0)
    {
      Serial2.read();
    }
 
}





void GPS2home(double lat,double lat_home,double lon,double lon_home,float psi, float *heading_home,float *dist_home_m)
{
  // 1 = current
  //2  = home

  float deg2rad = pi/180;



  // Haversine formula
  int E_radius_km = 6371;

  double dLat = (lat_home-lat)*deg2rad;
  double dLon = (lon_home-lon)*deg2rad;
  double lat_rad = lat*deg2rad;
  double lat_home_rad = lat_home*deg2rad;

  double a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat_rad) * cos(lat_home_rad); 
  double c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  *dist_home_m = float(E_radius_km * c * 1000);

  double y = sin(dLon) * cos(lat_home_rad);
  double x = cos(lat_rad)*sin(lat_home_rad) - sin(lat_rad)*cos(lat_home_rad)*cos(dLon);
  double bearing = atan2(y, x) / deg2rad;
  if(bearing < 0) bearing += 360;
  bearing = bearing - 180;//go towards home, not away
  if(bearing < 0) bearing += 360;
  bearing = bearing - psi + B_DECLINATION;
  if(bearing < 0) bearing += 360; 

  *heading_home = bearing;
}

