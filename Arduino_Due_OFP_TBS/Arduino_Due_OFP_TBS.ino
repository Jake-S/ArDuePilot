#include <Servo.h>
#include <SPI.h>
#include <SdFat.h>
#include <String.h>
#include <Wire.h>

#include "pins_map.h"
#include "calibration.h"
#include "pressure_temp.h"
#include "sound_notes.h"

#define pi 3.14159265359

void control_law(int ThrottleIn, int PitchIn, int RollIn, int YawIn, int FlapIn, float phi, float theta, float psi, float w_dps_xyz[3],float a_n_xyz[3],  int *motor_fr_cmd, int *motor_fl_cmd, int *motor_br_cmd,int *motor_bl_cmd);
int getGPSdata(int *gps_fix, int *num_sats, double *lat, double *lon, double *gps_alt_m, double *gps_gs_mph);
void GPSwarmBoot(float lat, float lon, int altitude);
void GPS2home(double lat,double lat_home,double lon,double lon_home,float psi,float *heading_home,float *dist_home_m);
void estimate_quaternions(float w_x,float w_y,float w_z,float a_x,float a_y,float a_z,float m_x,float m_y,float m_z,float SEq_1,float SEq_2,float SEq_3,float SEq_4,float w_error_last[3],float dt, float *q_new,float *w_error, int init);
void Quaternion2Euler(float q[4],float *phi,float *theta,float *psi);
void osd_display(int frame, float phi, float theta, float psi, int dt, int motor_armed, float throttle, int control_mode, float altitude_ft, float hdot, float ultra_altitude, float rssi, float batt_v, float batt_a, float dist_home, float dir_home, float lat, float lon, int gps_lock, int gps_sats);
int arming(int throttle_pos,int pitch_pos,int roll_pos,int yaw_pos,int flap_pos,float phi,float theta, int gps_fix,int gps_sats, float *altitude_home_ft, float altitude_ft, double *lat_home, double lat, double *lon_home, double lon);
void request_IMU_data();
void read_IMU_data(byte *a_bytes,byte *m_bytes,byte *w_bytes);
void unpack_IMU_data(byte *a_bytes,byte *m_bytes,byte *w_bytes, float *a_raw_data, float *m_raw_data,float *w_raw_data);
void cal_IMU_data(float *a_raw_data,float *m_raw_data,float *w_raw_data, float *a_n_xyz, float *m_n_xyz,float *w_dps_xyz);
float low_pass(float u, float y_last, float tau, float dt, float max_delta);

//One_off_switches
int sd_logging = 0;
int serial_output = 1;

// Motor Vars:
int motor_fr_cmd = 0;
int motor_fl_cmd = 0;
int motor_br_cmd = 0;
int motor_bl_cmd = 0;
Servo motor_FR; 
Servo motor_FL;
Servo motor_BR;
Servo motor_BL;

//Receiver Vars:
volatile boolean bNewSignal_throttle = false;
volatile boolean bNewSignal_roll = false; 
volatile boolean bNewSignal_pitch = false; 
volatile boolean bNewSignal_yaw = false; 
volatile boolean bNewSignal_gear = false; 
volatile boolean bNewSignal_flap = false; 
volatile unsigned long ulStartPeriod_throttle = 0; 
volatile unsigned long ulEndPeriod_throttle = 0;
volatile int ulStartPeriod_roll = 0; 
volatile int ulStartPeriod_pitch = 0; 
volatile int ulStartPeriod_yaw = 0;
volatile int ulStartPeriod_gear = 0; 
volatile int ulStartPeriod_flap = 0;
volatile int throttle_pos_v = 1000; // set in ISR, stored for use in throttle_poos
volatile int roll_pos_v = 1500; 
volatile int pitch_pos_v = 1500; 
volatile int yaw_pos_v = 1500; 
volatile int gear_pos_v = 1000; 
volatile int flap_pos_v = 1000; 
static int throttle_pos = 1000;
static int roll_pos = 1500;
static int pitch_pos = 1500;
static int yaw_pos = 1500;
static int gear_pos = 1000;
static int flap_pos = 1000;

// Pressure-Temperature Variables
const unsigned char OSS = 3;  // Oversampling Setting, samples per value
short int ac1_ptc;
short int ac2_ptc; 
short int ac3_ptc; 
unsigned int ac4_ptc;
unsigned int ac5_ptc;
unsigned int ac6_ptc;
short int b1_ptc; 
short int b2_ptc;
short int mb_ptc;
short int mc_ptc;
short int md_ptc;
long b5_ptc; 
short temperature;
unsigned int temp_uncal;
long pressure;
const float p0 = 101325;
float altitude_ft = 0;
float altitude_raw_ft = 0;
float altitude_home_ft = 0;
float altitude_agl_ft = 0;
int alt_t_us = 0;
int p_t1 = 0;
int p_t2 = 0;
int p_counter = 0;
int p_init = 0;
int p_error = 0;
int p_timeout_counter = 0;


// Sensor data:
byte a_bytes[6];
byte m_bytes[6];
byte w_bytes[6];
float a_raw_data[3];
float m_raw_data[3];
float w_raw_data[3];
float a_n_xyz[3];
float m_n_xyz[3];
float w_dps_xyz[3];

// IMU Filter vars:
float phi = 0.0;
float theta = 0.0;
float psi = 0.0;
float deltat = 0.0;
float t_last = 0.0;
float q[4] = {
  1,0,0,0};
float q_new[4] = {
  1, 0, 0, 0};
float w_error_last[3] = {
  0, 0, 0};
float w_error[3] = {
  0, 0, 0};

//GPS
int gps_fix = 0;
int gps_sats = 0;
double lat = 0;
double lon = 0;
double gps_alt_m = 0;
int gps_data = 0; // 0 no data, 1 = data, 2 = checksum error
double lat_home = 0;
double lon_home = 0;
float heading_home = 0;
float dist_home_m = 0;
double gps_gs_mph = 0;
//gear failsafe switch
volatile int gear_pers_count = 0;
int mode = 0;

//OSD vars 
int frame = 1;
int dt_step_array[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float rssi = 0;
int motors_armed = 0;
float throttle_percent = 0;
int dt_step = 0;
int voltage_analog_in = 0;
int current_analog_in = 0;
float batt_raw_v = 0.0;
float batt_v = 0.0; 
float batt_i = 0.0; 
float batt_raw_i = 0.0; 

//SD Card
char SD_file_name[25];
SdFat sd;
ofstream SDFile;
int buf_frame_counter = 0;


// Initialization
void setup() {
  
 // Serial.begin(57600); // USB Serial Output
  Serial1.begin(57600); // GPS
  Serial2.begin(57600); // OSD and TM
  Serial3.begin(57600); // IMU
  
  pinMode(GREEN_LED, OUTPUT);  
  pinMode(YELLOW_LED, OUTPUT);  
  pinMode(RED_LED, OUTPUT);  
  pinMode(BLUE_LED, OUTPUT);  
  pinMode(SOUND_PIN, OUTPUT); 
  
  pinMode(VOLTAGE_PIN_AN, INPUT);
  pinMode(CURRENT_PIN_AN, INPUT);
 // pinMode(RSSI_PIN_AN, INPUT);
  
  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_2_PIN, OUTPUT);
  pinMode(MOTOR_3_PIN, OUTPUT);
  pinMode(MOTOR_4_PIN, OUTPUT);
  
  
  //SD Card
  if(sd_logging)
  {
  sd.begin(SD_CARD_SPI_PIN, SPI_FULL_SPEED);
  int log_num = 1;
  while(1){
       String stringOne = "log_";
      String stringTwo = stringOne + log_num;
      String stringThree = stringTwo + ".txt";
      stringThree.toCharArray(SD_file_name, 25);
      if(sd.exists(SD_file_name)==1)
          log_num++;
      else
          break;
   }
  SDFile.open(SD_file_name, O_CREAT | O_WRITE | O_APPEND);
  }
     
     
  //Pressure-Temp Init
  Wire1.begin();
  delay(50);
  bmp085Calibration(&ac1_ptc, &ac2_ptc, &ac3_ptc, &ac4_ptc, &ac5_ptc, &ac6_ptc, &b1_ptc, &b2_ptc, &mb_ptc, &mc_ptc, &md_ptc);

  // Initialize IMU/quaternions
  request_IMU_data();
  read_IMU_data(a_bytes,m_bytes,w_bytes);
  unpack_IMU_data(a_bytes,m_bytes,w_bytes,a_raw_data,m_raw_data,w_raw_data);
  cal_IMU_data(a_raw_data,m_raw_data,w_raw_data,a_n_xyz,m_n_xyz,w_dps_xyz);
  deltat = 1/50;
  for(int i=1; i<500;i++)
  {
    estimate_quaternions(w_dps_xyz[0]*pi/180, w_dps_xyz[1]*pi/180, w_dps_xyz[2]*pi/180, a_n_xyz[0], a_n_xyz[1], a_n_xyz[2], m_n_xyz[0], m_n_xyz[1], m_n_xyz[2], q[0], q[1], q[2], q[3], w_error_last, deltat,  q_new, w_error, int(1));
    for(int j=0;j<4;j++) q[j] = q_new[j];
    delay(1);
  }

  // Receiver Interrupts:
  attachInterrupt(THROTTLE_PIN,ISR_throttle,CHANGE);
  attachInterrupt(AIL_PIN,ISR_roll,CHANGE);
  attachInterrupt(ELEV_PIN,ISR_pitch,CHANGE);
  attachInterrupt(RUDD_PIN,ISR_yaw,CHANGE);
  attachInterrupt(GEAR_PIN,ISR_gear,CHANGE);
  attachInterrupt(FLAP_PIN,ISR_flap,CHANGE);

  //Motor Servos
  motor_FR.attach(MOTOR_3_PIN);
  motor_FL.attach(MOTOR_4_PIN);
  motor_BR.attach(MOTOR_1_PIN);
  motor_BL.attach(MOTOR_2_PIN);

  delay(100);
  motor_FR.writeMicroseconds(0); 
  motor_FL.writeMicroseconds(0); 
  motor_BR.writeMicroseconds(0); 
  motor_BL.writeMicroseconds(0); 
  
  // Reboot GPS with initial coordinates
  GPSwarmBoot(float(34.688), float(-118.328), int(730));
}


// Run loop
void loop() {
  frame++;
  if(frame>10) frame = 1;
  
  
  if(p_error == 0)
  {
  p_t1 = millis();
  //Wire1.begin();
  //get Pressure Altitude
  if(frame == 1) bmp085RequestUT();
  if(frame == 4) temperature = bmp085GetTemperature(bmp085ReadUT(), ac5_ptc, ac6_ptc, mc_ptc, md_ptc, &b5_ptc);
  if(frame == 5) bmp085RequestUP(OSS);
  if(frame == 10) 
  {
  pressure = bmp085GetPressure(bmp085ReadUP(OSS), ac1_ptc, ac2_ptc, ac3_ptc, ac4_ptc,b1_ptc, b2_ptc, b5_ptc, OSS);
  altitude_raw_ft = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295))*3.28084;
    
  altitude_ft = low_pass(altitude_raw_ft, altitude_ft, float(1.5), deltat*10, float(20));
  altitude_agl_ft = altitude_ft - altitude_home_ft;
  }
  p_t2 = millis();
  }
  else
  {
    altitude_ft = altitude_home_ft+10;
    altitude_agl_ft = 10;
  }
  p_counter++;
  if(p_counter > 800)
    p_init = 1;
 if(p_init == 1 && ((p_t2-p_t1) > 20))
   p_timeout_counter++;
 if(p_timeout_counter > 4)
     p_error = 1;
    
  
  // Get GPS data
  gps_data = getGPSdata(&gps_fix, &gps_sats, &lat, &lon, &gps_alt_m, &gps_gs_mph);
  GPS2home(lat, lat_home, lon, lon_home, psi, &heading_home, &dist_home_m);

  //Get IMU Sensor Data
  request_IMU_data();
  read_IMU_data(a_bytes,m_bytes,w_bytes);
  unpack_IMU_data(a_bytes,m_bytes,w_bytes,a_raw_data,m_raw_data,w_raw_data);
  cal_IMU_data(a_raw_data,m_raw_data,w_raw_data,a_n_xyz,m_n_xyz,w_dps_xyz);
    
  //Run state estimation
  deltat = (micros()-t_last)/1000000;
  t_last = micros();
  estimate_quaternions(w_dps_xyz[0]*pi/180, w_dps_xyz[1]*pi/180, w_dps_xyz[2]*pi/180, a_n_xyz[0], a_n_xyz[1], a_n_xyz[2], m_n_xyz[0], m_n_xyz[1], m_n_xyz[2], q[0], q[1], q[2], q[3], w_error_last, deltat,  q_new, w_error, (int)0);
  for(int i=0;i<3;i++) w_error_last[i] = w_error[i];
  for(int i=0;i<4;i++) q[i] = q_new[i];
  Quaternion2Euler(q, &phi, &theta, &psi);
  phi += PHI_INSTALL_BIAS;
  theta += THETA_INSTALL_BIAS;


  //Update Recieved Control commands (disable ISRs quick)
  noInterrupts();
  if(bNewSignal_throttle)
  {
    throttle_pos = throttle_pos_v;
    bNewSignal_throttle = false;
  }
    if(bNewSignal_roll)
  {
    roll_pos = roll_pos_v;
    bNewSignal_roll = false;
  }
    if(bNewSignal_pitch)
  {
    pitch_pos = pitch_pos_v;
    bNewSignal_pitch = false;
  }
    if(bNewSignal_yaw)
  {
    yaw_pos = yaw_pos_v;
    bNewSignal_yaw = false;
  }
  if(bNewSignal_flap)
  {
    flap_pos = flap_pos_v;
    bNewSignal_flap = false;
  }
  interrupts();
  
  
  // Startup mode:
  mode = arming(throttle_pos, pitch_pos, roll_pos, yaw_pos, flap_pos, phi, theta,  gps_fix, gps_sats, &altitude_home_ft,  altitude_ft, &lat_home,  lat, &lon_home, lon);

 
  // Control Laws - Set motor commands
  if (mode == 4 && throttle_pos >= 1175) 
  {
     control_law(throttle_pos, pitch_pos, roll_pos, yaw_pos, flap_pos, phi, theta, psi, w_dps_xyz, a_n_xyz, &motor_fr_cmd, &motor_fl_cmd, &motor_br_cmd, &motor_bl_cmd);
  }
  else
  {
    motor_fr_cmd = 0;
    motor_fl_cmd = 0;
    motor_br_cmd = 0;
    motor_bl_cmd = 0;
  }


  // Send commands to ESCs
  if (gear_pers_count < 25)
  {
    motor_FR.writeMicroseconds(motor_fr_cmd); 
    motor_FL.writeMicroseconds(motor_fl_cmd); 
    motor_BR.writeMicroseconds(motor_br_cmd); 
    motor_BL.writeMicroseconds(motor_bl_cmd); 
  }
  else
  {  
    motor_FR.writeMicroseconds(0); 
    motor_FL.writeMicroseconds(0); 
    motor_BR.writeMicroseconds(0); 
    motor_BL.writeMicroseconds(0); 
  }



  // OSD Output
  // Place holder variables
   float hdot = 0;
   float ultra_altitude = 0;
   
   
   dt_step_array[frame-1] = (int)1/deltat;
   dt_step = dt_step_array[0];
   for(int i=1;i<10;i++)
   {
     if(dt_step_array[i] < dt_step) dt_step = dt_step_array[i];
   } // Worst Frame rate over last 10 frames
   
   throttle_percent = (float)((throttle_pos-1100.0)/(1925.0-1100.0))*100.0;
   
   if(throttle_pos>1150) {
      rssi = 255;  }
   else  {
      rssi = 0;  }
  
  if(mode >= 2 && gear_pers_count <= 25)  {
     motors_armed = 1; }
  else  {
     motors_armed = 0; }
  
  if(frame == 2)
  {
  analogReadResolution(12);
  voltage_analog_in = analogRead(VOLTAGE_PIN_AN);
  batt_raw_v = ((float)voltage_analog_in) * 0.01238570 + 0.36476789;
  batt_v = low_pass(batt_raw_v, batt_v, float(.2), deltat*10, float(3));
  
  current_analog_in = analogRead(CURRENT_PIN_AN);
  batt_raw_i = ((float)current_analog_in) * 0.01238570 + 0.36476789; // update this equation
  batt_i = low_pass(batt_raw_i, batt_i, float(.5), deltat*10, float(10));
  }

  //add p_error warning, check units
  osd_display(frame, phi, theta, psi, dt_step, motors_armed, throttle_percent, mode, altitude_agl_ft, hdot, gps_gs_mph, rssi, batt_v, batt_i, dist_home_m*3.28, heading_home, lat, lon, gps_fix, gps_sats);
 
  
  // SD Card data Logging
  if(sd_logging)
  {
  SDFile << "12345678" << "\n";
  if(frame == 7)
  {
    buf_frame_counter++;
    if(buf_frame_counter>=4)
    {
    SDFile << flush;
    buf_frame_counter = 1;
    }
  }
  }
  


  // Display data
 // if(serial_output)
 if(0)
  {
    /*
    Serial.print("Phi:");
    Serial.print(phi);
    Serial.print("\t");
    Serial.print("Theta:");
    Serial.print(theta);
    Serial.print("\t");
    Serial.print("Psi:");
    Serial.print(psi);
    Serial.print("\t");
    Serial.println();
   */
   
   /*
    Serial.print("Fix:");
    Serial.print(gps_fix);
    Serial.print("\t");
    Serial.print("Sats:");
    Serial.print(gps_sats);
    Serial.print("\t");
    Serial.print("lat:");
    Serial.print(lat,6);
    Serial.print("\t");
    Serial.print("lon:");
    Serial.print(lon,6);
    Serial.println();
    */
  
    /*
    Serial.print("a:");
    Serial.print(altitude_ft,1);
    Serial.print("\t");
    Serial.print("a:");
    Serial.print(altitude_raw_ft,1);
    Serial.print("\t");
    Serial.println();
   */

    
    Serial.print("t:");
    Serial.print(throttle_pos);
    Serial.print("\t");
    Serial.print("p:");
    Serial.print(pitch_pos);
    Serial.print("\t");
    Serial.print("r:");
    Serial.print(roll_pos);
    Serial.print("\t");
    Serial.print("y:");
    Serial.print(yaw_pos);
    Serial.print("\t");
    Serial.print("g:");
    Serial.print(gear_pos);
    Serial.print("\t");
    Serial.print("f:");
    Serial.print(flap_pos);
    Serial.print("\t");
    Serial.print("g count:");
    Serial.print(gear_pers_count);
    Serial.println();
   

   /*
    Serial.print("FR:");
    Serial.print(motor_fr_cmd);
    Serial.print("\t");
    Serial.print("FL:");
    Serial.print(motor_fl_cmd);
    Serial.print("\t");
    Serial.print("BL:");
    Serial.print(motor_bl_cmd);
    Serial.print("\t");
    Serial.print("BR:");
    Serial.print(motor_br_cmd);
    Serial.print("\t");
    Serial.println();
    */
    
    /*
     Serial.print("0:");
     Serial.print(w_dps_xyz[0]);
     Serial.print("\t");
     Serial.print("1:");
     Serial.print(w_dps_xyz[1]);
     Serial.print("\t");
     Serial.print("2:");
     Serial.print(w_dps_xyz[2]);
     Serial.println();
    */
    
    /*
    //Calibration Output
     Serial.println(9898.9898);
     Serial.println(a_raw_data[0]);
     Serial.println(a_raw_data[1]);
     Serial.println(a_raw_data[2]);
     Serial.println(m_raw_data[0]);
     Serial.println(m_raw_data[1]);
     Serial.println(m_raw_data[2]);
     Serial.println(w_raw_data[0]);
     Serial.println(w_raw_data[1]);
     Serial.println(w_raw_data[2]);
    */

  }

 
}








// Interrupt Routines
void ISR_throttle() {
  if(digitalRead(THROTTLE_PIN) == HIGH)
    ulStartPeriod_throttle = micros();
  else
  {
    if(ulStartPeriod_throttle && (bNewSignal_throttle == false))
    {
      throttle_pos_v = micros() - ulStartPeriod_throttle;
      ulStartPeriod_throttle = 0;
      bNewSignal_throttle = true;
}  }}
void ISR_roll() {
  if(digitalRead(AIL_PIN) == HIGH)
    ulStartPeriod_roll = micros();
  else
  {
    if(ulStartPeriod_roll && (bNewSignal_roll == false))
    {
      roll_pos_v = (int)(micros() - ulStartPeriod_roll);
      ulStartPeriod_roll = 0;
      bNewSignal_roll = true;
}  }}
void ISR_pitch() {
  if(digitalRead(ELEV_PIN) == HIGH)
    ulStartPeriod_pitch = micros();
  else
  {
    if(ulStartPeriod_pitch && (bNewSignal_pitch == false))
    {
      pitch_pos_v = (int)(micros() - ulStartPeriod_pitch);
      ulStartPeriod_pitch = 0;
      bNewSignal_pitch = true;
}  }}
void ISR_yaw() {
  if(digitalRead(RUDD_PIN) == HIGH)
    ulStartPeriod_yaw = micros();
  else
  {
    if(ulStartPeriod_yaw && (bNewSignal_yaw == false))
    {
      yaw_pos_v = (int)(micros() - ulStartPeriod_yaw);
      ulStartPeriod_yaw = 0;
      bNewSignal_yaw = true;
}  }}
void ISR_gear() {
  if(digitalRead(GEAR_PIN) == HIGH)
    ulStartPeriod_gear = micros();
  else
  {
    if(ulStartPeriod_gear)
    {
      gear_pos_v = (int)(micros() - ulStartPeriod_gear);
      ulStartPeriod_gear = 0;
   
      if(gear_pos_v <1300)
      {
       if(gear_pers_count < 28) gear_pers_count++;
      }
      else
      {
      if(gear_pers_count > 0) gear_pers_count--;
      }
      
      if (gear_pers_count > 25)
      {
          motor_FR.writeMicroseconds(0); 
          motor_FL.writeMicroseconds(0); 
          motor_BR.writeMicroseconds(0); 
          motor_BL.writeMicroseconds(0); 
      }
    }    
  }}
void ISR_flap() {
  if(digitalRead(FLAP_PIN) == HIGH)
    ulStartPeriod_flap = micros();
  else
  {
    if(ulStartPeriod_flap && (bNewSignal_flap == false))
    {
      flap_pos_v = (int)(micros() - ulStartPeriod_flap);
      ulStartPeriod_flap = 0;
      bNewSignal_flap = true;
}  }}

