

#include <Arduino.h>

static float theta_int;
static float phi_int;

void control_law(int ThrottleIn, int PitchIn, int RollIn, int YawIn, int FlapIn, float phi, float theta, float psi, float w_dps_xyz[3],float a_n_xyz[3], int *motor_fr_cmd, int *motor_fl_cmd, int *motor_br_cmd,int *motor_bl_cmd)
{

  float yaw_sign = 1;
 
  float theta_Kp = 1.18;
  float phi_Kp   = 1.18;
  float theta_Ki = 0.0;
  float phi_Ki   = 0.0;
  
  float p_Kp = -0.58;
  float q_Kp = -0.58;
  float r_Kp = 1.62;
  
  float theta_cmd;
  float phi_cmd;
  float theta_err;
  float phi_err;
  float pitch_cmd = 0;
  float roll_cmd = 0;
  float yaw_cmd = 0;
  
  float p = w_dps_xyz[0];
  float q = w_dps_xyz[1];
  float r = w_dps_xyz[2];


  // Input Limiters
  if (ThrottleIn >= 1925)  ThrottleIn = 1925;
  if (ThrottleIn < 1000)  ThrottleIn = 1000;
  if (PitchIn >= 2000)  PitchIn = 2000;
  if (PitchIn < 1000)  PitchIn = 1000;
  if (RollIn >= 2000)  RollIn = 2000;
  if (RollIn < 1000)  RollIn = 1000;
  if (YawIn >= 2000)  YawIn = 2000;
  if (YawIn < 1000)  YawIn = 1000;
  
  // Stick Shaping
  theta_cmd = -1.2*(0.0000005667*pow((float)PitchIn,3)-0.0026106873*pow((float)PitchIn,2) + 4.1236831405*(float)PitchIn - 2227.0978);
  phi_cmd   = -1.2*(0.0000005667*pow((float)RollIn,3)-0.0026106873*pow((float)RollIn,2) + 4.1236831405*(float)RollIn - 2227.0978);
  
  YawIn -= 1526;
  yaw_cmd = -(float)YawIn/1.35;
  
  
  // Angle Error
  theta_err =  theta - theta_cmd;
  phi_err = phi - phi_cmd;
  
 
 // Angle Integration
  theta_int = 0;
  phi_int = 0;
  
  // Command
  pitch_cmd = -(theta_err*theta_Kp + theta_int*theta_Ki) + q*q_Kp;
  roll_cmd =  phi_err*phi_Kp + phi_int*phi_Ki - p*p_Kp;
  yaw_cmd = yaw_sign * (r - yaw_cmd) * r_Kp ;
 
   int motor_fr = ThrottleIn + ( pitch_cmd + roll_cmd + yaw_cmd);
   int motor_fl = ThrottleIn + ( pitch_cmd - roll_cmd - yaw_cmd);
   int motor_br = ThrottleIn + (-pitch_cmd + roll_cmd - yaw_cmd);
   int motor_bl = ThrottleIn + (-pitch_cmd - roll_cmd + yaw_cmd);
   
   
   //Output Formating
   if(motor_fr > 2000) motor_fr = 2000;
   if(motor_fr < 1000) motor_fr = 1000;
   if(motor_fl > 2000) motor_fl = 2000;
   if(motor_fl < 1000) motor_fl = 1000;
   if(motor_br > 2000) motor_br = 2000;
   if(motor_br < 1000) motor_br = 1000;
   if(motor_bl > 2000) motor_bl = 2000;
   if(motor_bl < 1000) motor_bl = 1000;

  *motor_fr_cmd = motor_fr;
  *motor_fl_cmd = motor_fl;
  *motor_br_cmd = motor_br;
  *motor_bl_cmd = motor_bl;
}




