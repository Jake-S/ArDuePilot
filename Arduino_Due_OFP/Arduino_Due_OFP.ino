
#include <Servo.h>
#include <Wire.h>

#include "pins_map.h"
#include "sensor_data.h"
#include "imu.h"
#include "calibration.h"
#include "CLaw.h"
#include "pressure_temp.h"
#include "sound_notes.h"
#include "LEDs.h"
#include "osd.h"

#define pi 3.14159265359


// Temporary variables (testing)
float t1;
float t2;

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
long pressure;
const float p0 = 101325;
float altitude;

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

//CLaws Vars:

//Safety
int engage = 0;
int counter_arm1 = 0;
int counter_arm2 = 0;
int green_led = 0;
int yellow_led = 0;
int red_led = 0;
//gear failsafe switch
volatile int gear_pers_count;


//Serial output
int output_i = 0;



// Initialization
void setup() {
  
  Serial.begin(57600);
    
  Serial2.begin(57600);
  
  pinMode(GREEN_LED, OUTPUT);  
  pinMode(YELLOW_LED, OUTPUT);  
  pinMode(RED_LED, OUTPUT);  
  pinMode(SOUND_PIN, OUTPUT); 
  
  


  //Pressure-Temp Init
  //Wire.begin();
 // bmp085Calibration(&ac1_ptc, &ac2_ptc, &ac3_ptc, &ac4_ptc, &ac5_ptc, &ac6_ptc, &b1_ptc, &b2_ptc, &mb_ptc, &mc_ptc, &md_ptc);


  // Initialize IMU/quaternions
  request_IMU_data();
  read_IMU_data(a_bytes,m_bytes,w_bytes);
  unpack_IMU_data(a_bytes,m_bytes,w_bytes,a_raw_data,m_raw_data,w_raw_data);
  cal_IMU_data(a_raw_data,m_raw_data,w_raw_data,a_n_xyz,m_n_xyz,w_dps_xyz);
  deltat = 1/50;
  
  for(int i=1; i<100;i++)
  {
    estimate_quaternions(w_dps_xyz[0]*pi/180, w_dps_xyz[1]*pi/180, w_dps_xyz[2]*pi/180, a_n_xyz[0], a_n_xyz[1], a_n_xyz[2], m_n_xyz[0], m_n_xyz[1], m_n_xyz[2], q[0], q[1], q[2], q[3], w_error_last, deltat,  q_new, w_error, int(1));
    for(int j=0;j<4;j++) q[j] = q_new[j];
    delay(25);
  }


  // Receiver Interrupts:
  attachInterrupt(THROTTLE_PIN,ISR_throttle,CHANGE);
  attachInterrupt(AIL_PIN,ISR_roll,CHANGE);
  attachInterrupt(ELEV_PIN,ISR_pitch,CHANGE);
  attachInterrupt(RUDD_PIN,ISR_yaw,CHANGE);
  attachInterrupt(GEAR_PIN,ISR_gear,CHANGE);
  attachInterrupt(FLAP_PIN,ISR_flap,CHANGE);

  //Motor Servos
  motor_FR.attach(MOTOR_4_PIN);
  motor_FL.attach(MOTOR_1_PIN);
  motor_BR.attach(MOTOR_3_PIN);
  motor_BL.attach(MOTOR_2_PIN);

  delay(50);
  motor_FR.writeMicroseconds(0); 
  motor_FL.writeMicroseconds(0); 
  motor_BR.writeMicroseconds(0); 
  motor_BL.writeMicroseconds(0); 
  
  delay(250);
  green_led = 1;
  gear_pers_count = 0;
}


// Run loop
void loop() {

  //get Pressure Altitude
  /*
  temperature = bmp085GetTemperature(bmp085ReadUT(), ac5_ptc, ac6_ptc, mc_ptc, md_ptc, &b5_ptc);
  pressure = bmp085GetPressure(bmp085ReadUP(OSS), ac1_ptc, ac2_ptc, ac3_ptc, ac4_ptc,b1_ptc, b2_ptc, b5_ptc, OSS);
  altitude = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295))*3.28084;
  */
  
  //Get Sensor Data
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



  //Update Input commands (disable ISRs quick)
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
  


  // Arm the system by pumping throttle (move to function)
  if (counter_arm1 == 60)
   SoundNoTimer(SOUND_PIN,300,NOTE_B5);
  if (counter_arm2 == 60)
    SoundNoTimer(SOUND_PIN,300,NOTE_B5);
    
  if ((engage == 0) && (throttle_pos < 1250))
    counter_arm1++;
  if ((engage == 0) && (counter_arm1 > 60) && (throttle_pos > 1250) && (flap_pos < 1700))
    counter_arm2++;
  if ((engage == 0) && (counter_arm2 > 60) && (throttle_pos < 1150))
  {
    engage = 1;
    SoundNoTimer(SOUND_PIN,300,NOTE_C4);
    delay(600);
    SoundNoTimer(SOUND_PIN,300,NOTE_C4);
    green_led = 0;
  }


  // Control Laws
  if (engage == 1) 
  {
    if (throttle_pos >= 1200)
    {
      if(flap_pos < 1700)
      {
        // Attitude control laws
        control_law(throttle_pos, pitch_pos, roll_pos, yaw_pos, flap_pos, phi, theta, psi, w_dps_xyz, a_n_xyz, &motor_fr_cmd, &motor_fl_cmd, &motor_br_cmd, &motor_bl_cmd);
      }
      else
      {
        // Aerobatic - rate control laws
        control_law_rate(throttle_pos, pitch_pos, roll_pos, yaw_pos, flap_pos, phi, theta, psi, w_dps_xyz, a_n_xyz, &motor_fr_cmd, &motor_fl_cmd, &motor_br_cmd, &motor_bl_cmd);
      }
      
      red_led = 1;
    }
     else 
    {
    motor_fr_cmd = 0;
    motor_fl_cmd = 0;
    motor_br_cmd = 0;
    motor_bl_cmd = 0;
    red_led = 0;
    }
    yellow_led = 1;
  }
  else
  {
    motor_fr_cmd = 0;
    motor_fl_cmd = 0;
    motor_br_cmd = 0;
    motor_bl_cmd = 0;
    red_led = 0;
    //yellow_led = 0;
  }


  // Send commands to ESCs
  if (gear_pers_count > 25)
  {
    motor_FR.writeMicroseconds(0); 
    motor_FL.writeMicroseconds(0); 
    motor_BR.writeMicroseconds(0); 
    motor_BL.writeMicroseconds(0); 
  }
  else
  {
    motor_FR.writeMicroseconds(motor_fr_cmd); 
    motor_FL.writeMicroseconds(motor_fl_cmd); 
    motor_BR.writeMicroseconds(motor_br_cmd); 
    motor_BL.writeMicroseconds(motor_bl_cmd); 
  }


    

 // Caution
  if(motor_fr_cmd > 1200 || motor_fl_cmd > 1200 || motor_br_cmd > 1200 || motor_bl_cmd > 1200)
  {
        green_led = 0;
        red_led = 1;
        yellow_led = 1;
  }  
        

// LEDs
 controlLEDs(green_led, yellow_led, red_led);



  osd_display(theta);


  // Display data
  if (output_i == 1)
  {

    

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
    

/*
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
      Serial.println();
       */
    
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
      Serial.print(gear_pers_count);
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

 // delay(5);
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
   
      // Gear failsafe switch, if down for >~1 second command motors in intertupt to 0, allows for engine cutoff if stuck in infinite loop in code
      if(gear_pos_v <1300)
      {
      gear_pers_count++;
      }
      else
      {
      gear_pers_count = 0;
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

