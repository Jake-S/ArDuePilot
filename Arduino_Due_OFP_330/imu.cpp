// Math library required for `sqrt'
#include <math.h>
#include "imu.h"
#include "calibration.h"

#define pi 3.1415926



void estimate_quaternions(float w_x,float w_y,float w_z,float a_x,float a_y,float a_z,float m_x,float m_y,float m_z,float SEq_1,float SEq_2,float SEq_3,float SEq_4,float w_error_last[3],float dt, float *q_new,float *w_error, int init)
{
  // local vars
  float nu = .02;
  float norm; // vector norm
  float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
  float f_1, f_2, f_3, f_4, f_5, f_6; 
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
  float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
  float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)

  // axulirary variables to avoid reapeated calcualtions
  float halfSEq_1 = 0.5f * SEq_1;
  float halfSEq_2 = 0.5f * SEq_2;
  float halfSEq_3 = 0.5f * SEq_3;
  float halfSEq_4 = 0.5f * SEq_4;
  float twoSEq_1 = 2.0f * SEq_1;
  float twoSEq_2 = 2.0f * SEq_2;
  float twoSEq_3 = 2.0f * SEq_3;
  float twoSEq_4 = 2.0f * SEq_4;
  float SEq_1SEq_3 = SEq_1 * SEq_3;
  float SEq_2SEq_4 = SEq_2 * SEq_4;
  
  float twob_x = 2.0f * B_X;
  float twob_z = 2.0f * B_Z;
  float twob_xSEq_1 = 2.0f * B_X * SEq_1;
  float twob_xSEq_2 = 2.0f * B_X * SEq_2;
  float twob_xSEq_3 = 2.0f * B_X * SEq_3;
  float twob_xSEq_4 = 2.0f * B_X * SEq_4;
  float twob_zSEq_1 = 2.0f * B_Z * SEq_1;
  float twob_zSEq_2 = 2.0f * B_Z * SEq_2;
  float twob_zSEq_3 = 2.0f * B_Z * SEq_3;
  float twob_zSEq_4 = 2.0f * B_Z * SEq_4;
  
  // normalise the accelerometer measurement
  norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  a_x /= norm;
  a_y /= norm;
  a_z /= norm;
  // normalise the magnetometer measurement
  norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
  m_x /= norm;
  m_y /= norm;
  m_z /= norm;
  
  // compute the objective function and Jacobian
  f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
  f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
  f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
  f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
  f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
  f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
  J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
  J_12or23 = 2.0f * SEq_4;
  J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21; // negated in matrix multiplication
  J_33 = 2.0f * J_11or24; // negated in matrix multiplication
  J_41 = twob_zSEq_3; // negated in matrix multiplication
  J_42 = twob_zSEq_4;
  J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
  J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
  J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
  J_52 = twob_xSEq_3 + twob_zSEq_1;
  J_53 = twob_xSEq_2 + twob_zSEq_4;
  J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
  J_61 = twob_xSEq_3;
  J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
  J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
  J_64 = twob_xSEq_2;
  // compute the gradient (matrix multiplication)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
  // normalise the gradient to estimate direction of the gyroscope error
  norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
  SEqHatDot_1 = SEqHatDot_1 / norm;
  SEqHatDot_2 = SEqHatDot_2 / norm;
  SEqHatDot_3 = SEqHatDot_3 / norm;
  SEqHatDot_4 = SEqHatDot_4 / norm;
  // compute angular estimated direction of the gyroscope error
  w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
  w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
  w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
  // compute and remove the gyroscope baises
  w_error[0] = w_error_last[0] + w_err_x * dt * zeta;
  w_error[1] = w_error_last[1] + w_err_y * dt * zeta;
  w_error[2] = w_error_last[2] + w_err_z * dt * zeta;
  w_x -= w_error[0] ;
  w_y -= w_error[1];
  w_z -= w_error[2];
  // compute the quaternion rate measured by gyroscopes
  SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
  SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
  SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
  SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
  // compute then integrate the estimated quaternion rate
  if(init<.5){
  SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
  SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
  SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
  SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;
  norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
  //normalize quaternions
  q_new[0] = SEq_1/norm;
  q_new[1] = SEq_2/norm;
  q_new[2] = SEq_3/norm;
  q_new[3] = SEq_4/norm;
  }
  else
  {
  q_new[0] = SEq_1 - nu*SEqHatDot_1;
  q_new[1] = SEq_2 - nu*SEqHatDot_2;
  q_new[2] = SEq_3 - nu*SEqHatDot_3;
  q_new[3] = SEq_4 - nu*SEqHatDot_4;
  }
  
  /* 
  // compute flux in the earth frame
  // float h_x, h_y, h_z; // computed flux in the earth frame
  SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
  SEq_1SEq_3 = SEq_1 * SEq_3;
  SEq_1SEq_4 = SEq_1 * SEq_4;
  SEq_3SEq_4 = SEq_3 * SEq_4;
  SEq_2SEq_3 = SEq_2 * SEq_3;
  SEq_2SEq_4 = SEq_2 * SEq_4;
  h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
  h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
  h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
  // normalise the flux vector to have only components in the x and z
  b_x = sqrt((h_x * h_x) + (h_y * h_y));
  b_z = h_z;
  */
}

  
  
void Quaternion2Euler(float q[4],float *phi,float *theta, float *psi)
{
  *phi = (float)180/pi*atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
  *theta = (float)180/pi*asin(2*(q[0]*q[2]-q[3]*q[1]));
  *psi = (float)180/pi*atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3])) + B_DECLINATION;
  if (*psi<0) *psi += 360;
}

/*
void Euler2Quaterion(float phi,float theta,float psi, float *q)
 {
 //phi theta psi in radians
 phi_2 = phi/2;
 theta_2 = theta/2;
 psi_2 = psi/2;
 
 q = [cos(phi_2)*cos(theta_2)*cos(psi_2) + sin(phi_2)*sin(theta_2)*sin(psi_2); ...
 sin(phi_2)*cos(theta_2)*cos(psi_2) - cos(phi_2)*sin(theta_2)*sin(psi_2); ...
 cos(phi_2)*sin(theta_2)*cos(psi_2) + sin(phi_2)*cos(theta_2)*sin(psi_2); ...
 cos(phi_2)*cos(theta_2)*sin(psi_2) - sin(phi_2)*sin(theta_2)*cos(psi_2);];
 }
 */

