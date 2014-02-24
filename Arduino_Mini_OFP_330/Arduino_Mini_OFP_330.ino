#include <Wire.h>
#include <Arduino.h>
#include "calibration.h"
#include "pressure_temp.h"

#define pi 3.14159265359

// Pins
#define VOLTAGE_PIN_AN A1

// I2C Defines:
// ACCEL
#define ADXL345_ADDRESS (0xA6 >> 1)
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

// Protos:
void unpack_IMU_data(byte *a_bytes,byte *m_bytes,byte *w_bytes, float *a_raw_data, float *m_raw_data,float *w_raw_data);
void cal_IMU_data(float *a_raw_data,float *m_raw_data,float *w_raw_data, float *a_n_xyz, float *m_n_xyz,float *w_dps_xyz);
float low_pass(float u, float y_last, float tau, float dt, float max_delta);
void estimate_quaternions(float w_x,float w_y,float w_z,float a_x,float a_y,float a_z,float m_x,float m_y,float m_z,float SEq_1,float SEq_2,float SEq_3,float SEq_4,float w_error_last[3],float dt, float *q_new,float *w_error, int init);
void Quaternion2Euler(float q[4],float *phi,float *theta,float *psi);
void i2c_read(int address, byte reg, int count, byte* data);
void i2c_write(int address, byte reg, byte data);

// Gyro, Accel, Magnetometer Variables
byte a_bytes[6];
byte m_bytes[6];
byte w_bytes[6];
float a_raw_data[3];
float m_raw_data[3];
float w_raw_data[3];
float a_n_xyz[3];
float m_n_xyz[3];
float w_dps_xyz[3];


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
float hdot = 0;

// IMU Variables:
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

// Voltage Measurement Variables
int voltage_analog_in = 0;
float batt_raw_v = 0.0;
float batt_v = 0.0; 
int loop_counter = 0;

// OSD variables
uint8_t first_half;  
uint8_t sencond_half; 
uint16_t osd_buffer;
uint16_t spare = 0;

// Other
int frame = 1;

void setup() {
  Wire.begin();

  //CHANGE TO LOWER
  Serial.begin(19200);

  delay(100);
  i2c_write(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, ADXL_PWRCTL_MEASURE);
  delay(500);
  i2c_write(ADXL345_ADDRESS, (0x31), (0x4E)); //Set to +/-8g mode : 01001110
  delay(100); 
  i2c_write(HMC5843_ADDRESS, HMC5843_REGISTER_MEASMODE, HMC5843_MEASMODE_CONT);
  delay(100);
  i2c_write(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, ITG3200_FULLSCALE | ITG3200_42HZ);
  delay(100);
  bmp085Calibration(&ac1_ptc, &ac2_ptc, &ac3_ptc, &ac4_ptc, &ac5_ptc, &ac6_ptc, &b1_ptc, &b2_ptc, &mb_ptc, &mc_ptc, &md_ptc);
  delay(100);
  
  
 

}

void loop() {
  loop_counter++;
  frame++;
  if(frame>10) frame = 1;


  // Get Pressure-Temp => Altitude
  if(frame == 4) 
  {
   bmp085RequestUT();
   delay(20);
   temperature = bmp085GetTemperature(bmp085ReadUT(), ac5_ptc, ac6_ptc, mc_ptc, md_ptc, &b5_ptc);
  }
   if(frame == 10) 
   {
   bmp085RequestUP(OSS);
   delay(20);
   pressure = bmp085GetPressure(bmp085ReadUP(OSS), ac1_ptc, ac2_ptc, ac3_ptc, ac4_ptc,b1_ptc, b2_ptc, b5_ptc, OSS);
   altitude_raw_ft = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295))*3.28084;
   altitude_ft = low_pass(altitude_raw_ft, altitude_ft, float(3), deltat*10, float(20));
   altitude_agl_ft = altitude_ft - altitude_home_ft;
   hdot = 0;
  }
if(loop_counter > 1000)
{
altitude_home_ft = altitude_ft;
}

  //Get IMU Sensor Data
  i2c_read(ADXL345_ADDRESS, ADXL345_REGISTER_XLSB, 6, a_bytes);
  i2c_read(HMC5843_ADDRESS, HMC5843_REGISTER_XMSB, 6, m_bytes);
  i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_XMSB, 6, w_bytes);
  unpack_IMU_data(a_bytes,m_bytes,w_bytes, a_raw_data, m_raw_data, w_raw_data);
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

  // Read Batt Voltage
  if(frame == 9)
  {
  //voltage_analog_in = analogRead(VOLTAGE_PIN_AN);
  batt_raw_v = ((float)voltage_analog_in) * 0.01238570 + 0.36476789;
  batt_v = low_pass(batt_raw_v, batt_v, float(.2), deltat*10, float(3));
  }
  
  
  
  
  
  // Write data to OSD
  if(frame <= 8 && frame != 4)
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
  else if(frame == 9)
  {
    Serial.write(B10000100);
    Serial.write(B10000100);
    osd_buffer = (short int)(250); //rssi
    first_half   = osd_buffer >> 8;   // >>>> >>>> ########
    sencond_half = osd_buffer & B11111111; // ######## ________
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(batt_v*100);
    first_half   = osd_buffer >> 8;  
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(0*100); //batt_i
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
  }
  else if(frame == 10)
  {
    Serial.write(B10000011);
    Serial.write(B10000011);
    osd_buffer = (short int)(altitude_agl_ft);
    first_half   = osd_buffer >> 8;   // >>>> >>>> ########
    sencond_half = osd_buffer & B11111111; // ######## ________
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(hdot*100);
    first_half   = osd_buffer >> 8;  
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
    osd_buffer = (short int)(0);
    first_half   = osd_buffer >> 8;
    sencond_half = osd_buffer & B11111111;
    Serial.write(first_half);
    Serial.write(sencond_half);
  }


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
    Serial.print("Alt:");
    Serial.print(altitude_ft);
    Serial.print("\t");
        Serial.println();
        */
    /*
    Serial.print("Temp:");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print("Pressue:");
    Serial.print(pressure);
    Serial.print("\t");
    Serial.println();
    */
 
}



