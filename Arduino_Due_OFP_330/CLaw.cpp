
#include "CLaw.h"
#include <arduino.h>

static float theta_int;
static float phi_int;

void control_law(int ThrottleIn, int PitchIn, int RollIn, int YawIn, int FlapIn, float phi, float theta, float psi, float w_dps_xyz[3],float a_n_xyz[3], int *motor_fr_cmd, int *motor_fl_cmd, int *motor_br_cmd,int *motor_bl_cmd)
{

  float theta_Kp = 1.2;
  float phi_Kp   = 1.2;
  float theta_Ki = 0.0;
  float phi_Ki   = 0.0;
  
  float p_Kp = -0.6;
  float q_Kp = -0.6;
  float r_Kp = 1.3;
  
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


  
  if (ThrottleIn >= 1925)  ThrottleIn = 1925; // Limit max throttle cmd to allow for an extra 50ms above that for control
  
  // Stick Shaping
  theta_cmd = -1.1*(0.0000005667*pow((float)PitchIn,3)-0.0026106873*pow((float)PitchIn,2) + 4.1236831405*(float)PitchIn - 2227.0978);
  phi_cmd   = -1.1*(0.0000005667*pow((float)RollIn,3)-0.0026106873*pow((float)RollIn,2) + 4.1236831405*(float)RollIn - 2227.0978);
  
  YawIn -= 1526;
  yaw_cmd = -(float)YawIn/1.25;
  
  
  // Angle Error
  theta_err =  theta - theta_cmd;
  phi_err = phi - phi_cmd;
  
 
 // Angle Integration
  theta_int = 0;
  phi_int = 0;
  
  // Command
  pitch_cmd = -(theta_err*theta_Kp + theta_int*theta_Ki) + q*q_Kp;
  roll_cmd =  phi_err*phi_Kp + phi_int*phi_Ki - p*p_Kp;
  yaw_cmd = (r - yaw_cmd) * r_Kp ;
 

  *motor_fr_cmd = ThrottleIn + ( pitch_cmd + roll_cmd + yaw_cmd);
  *motor_fl_cmd = ThrottleIn + ( pitch_cmd - roll_cmd - yaw_cmd);
  *motor_br_cmd = ThrottleIn + (-pitch_cmd + roll_cmd - yaw_cmd);
  *motor_bl_cmd = ThrottleIn + (-pitch_cmd - roll_cmd + yaw_cmd);

}




void control_law_rate(int ThrottleIn, int PitchIn, int RollIn, int YawIn, int FlapIn, float phi, float theta, float psi, float w_dps_xyz[3],float a_n_xyz[3], int *motor_fr_cmd, int *motor_fl_cmd, int *motor_br_cmd,int *motor_bl_cmd)
{
  float p = w_dps_xyz[0];
  float q = w_dps_xyz[1];
  float r = w_dps_xyz[2];
  
  float p_Kp = .75;
  float q_Kp = .75;
  float r_Kp = 1.3;
  
  float pitch_cmd = 0;
  float roll_cmd = 0;
  float yaw_cmd = 0;
   float q_cmd = 0;
  float p_cmd = 0;
  float r_cmd = 0;

  if (ThrottleIn >= 1925)  ThrottleIn = 1925;
  
  // Stick Shaping
  q_cmd = -4.0*(0.0000005667*pow((float)PitchIn,3)-0.0026106873*pow((float)PitchIn,2) + 4.1236831405*(float)PitchIn - 2227.0978);
  p_cmd   = -4.0*(0.0000005667*pow((float)RollIn,3)-0.0026106873*pow((float)RollIn,2) + 4.1236831405*(float)RollIn - 2227.0978);
  YawIn -= 1526;
  r_cmd = -(float)YawIn/1.0;

  // Command
  pitch_cmd = -(q - q_cmd) * q_Kp ;
  roll_cmd =  (p - p_cmd) * p_Kp ;
  yaw_cmd =   (r - r_cmd) * r_Kp ;
 
  *motor_fr_cmd = ThrottleIn + ( pitch_cmd + roll_cmd + yaw_cmd);
  *motor_fl_cmd = ThrottleIn + ( pitch_cmd - roll_cmd - yaw_cmd);
  *motor_br_cmd = ThrottleIn + (-pitch_cmd + roll_cmd - yaw_cmd);
  *motor_bl_cmd = ThrottleIn + (-pitch_cmd - roll_cmd + yaw_cmd);
}

