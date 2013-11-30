/*
#include <math.h>
#include "imu.h"


void filtUpdate(float w_x,float w_y,float w_z,float a_x,float a_y,float a_z,float m_x,float m_y,float m_z,float q_1,float q_2,float q_3,float q_4,float P_last[4][4],float dt, float *q_new,float *P1_new,float *P2_new,float *P3_new,float *P4_new)
{
  float b[3] = {0.5081, 0, 0.8613};

  // Tuning Parameters:
  float Q_i = .3;
  float R_i = 15;
  float mu = .05;
  
 

  // Kalman Filter Parms:
  float H[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  float Pk[4][4]={{0,0,0,0},{ 0,0,0,0},{ 0,0,0,0} ,{0,0,0,0}};
  float Fk[4][4]={{0,0,0,0},{ 0,0,0,0},{ 0,0,0,0} ,{0,0,0,0}};
  float S[4][4]={{0,0,0,0},{ 0,0,0,0},{ 0,0,0,0} ,{0,0,0,0}};
  float K[4][4]={{0,0,0,0},{ 0,0,0,0},{ 0,0,0,0} ,{0,0,0,0}};
  float xk[4] = {0,0,0,0};
  float zk[4] = {0,0,0,0};
  float yk[4] = {0,0,0,0};
  float Q[4][4] = {
    {Q_i,0,0,0}
   ,{0,Q_i,0,0}
   ,{0,0,Q_i,0}
   ,{0,0,0,Q_i}  }; // gyro process noise covariance
  float R[4][4] = {
    {R_i,0,0,0}
   ,{0,R_i,0,0}
   ,{0,0,R_i,0}
   ,{0,0,0,R_i}}; // sensor noise covariance
  
  // Predict states based on gyro
  float dt_2 = dt/2;
  Fk[0][0] = 1;
  Fk[1][0] = dt_2*w_x;
  Fk[2][0] = dt_2*w_y;
  Fk[3][0] = dt_2*w_z;
  Fk[0][1] = -dt_2*w_x;
  Fk[1][1] = 1;
  Fk[2][1] = -dt_2*w_z;
  Fk[3][1] = dt_2*w_y;
  Fk[0][2] = -dt_2*w_y;
  Fk[1][2] = dt_2*w_z;
  Fk[2][2] = 1;
  Fk[3][2] = -dt_2*w_x;
  Fk[0][3] = -dt_2*w_z;
  Fk[1][3] = -dt_2*w_y;
  Fk[2][3] = dt_2*w_x;
  Fk[3][3] = 1;
  //xk = Fk*q
  
  for(int i = 0; i < 4; i++){
    xk[i] = Fk[i][0]*q_1 +  Fk[i][1]*q_2 + Fk[i][2]*q_3 + Fk[i][3]*q_4;
  }
  // Pk = Fk*Pk_last*Fk' + Q
  float tmp[4][4] = {{0,0,0,0},{ 0,0,0,0},{ 0,0,0,0} ,{0,0,0,0}};
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      for(int k = 0; k < 4; k++){
        tmp[i][j] += Fk[i][k] * P_last[k][j];
      }
    }
  }
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      for(int k = 0; k < 4; k++){
        Pk[i][j] += tmp[i][k] * Fk[j][k]; 
      }
      Pk[i][j] += Q[i][j];
    }
  }


  // Determine states from accel/mag 
  //q_dt = 0.5*(quaternion_cross(q_last,[1, w'])); 
  //q_dt_norm = norm(q_dt);
  //mu = mu_k*q_dt_norm*dt;
  float f[6] = {0,0,0,0,0,0};
  float J[6][4];
  float Df[4] = {0,0,0,0};
  float norm_df = 0;
  float norm_q = 0;
  float q_result[4] = {0,0,0,0};
  // Gradient Descent method:
  for(int i=0;i<=3;i++){
    f[0]=2*(q_2*q_4-q_1*q_3)-a_x;
    f[1]=2*(q_1*q_2+q_3*q_4)-a_y;
    f[2]=2*(0.5-q_2*q_2-q_3*q_3)-a_z;
    J[0][0] = -2*q_3;
    J[0][1] = 2*q_4; 
    J[0][2] = -2*q_1;
    J[0][3] = 2*q_2;
    J[1][0] = 2*q_2;
    J[1][1] = 2*q_1; 
    J[1][2] = 2*q_4;
    J[1][3] = 2*q_3;
    J[2][0] = 0; 
    J[2][1] = -4*q_2; 
    J[2][2] = -4*q_3; 
    J[2][3] = 0;
    f[3]=2*b[0]*(0.5-q_3*q_3-q_4*q_4)+2*b[2]*(q_2*q_4-q_1*q_3)-m_x;
    f[4]=(2*b[0]*(q_2*q_3-q_1*q_4)+2*b[2]*(q_1*q_2+q_3*q_4)-m_y);
    f[5]=2*b[0]*(q_1*q_3+q_2*q_4)+2*b[2]*(0.5-q_2*q_2-q_3*q_3)-m_z;
    J[3][0] = -2*b[2]*q_3;
    J[3][1] = 2*b[2]*q_4;
    J[3][2] = -4*b[0]*q_3-2*b[2]*q_1;
    J[3][3] = -4*b[0]*q_4+2*b[2]*q_2;
    J[4][0] = -2*b[0]*q_4+2*b[2]*q_2;
    J[4][1] = 2*b[0]*q_3+2*b[2]*q_1;
    J[4][2] = 2*b[0]*q_2+2*b[2]*q_4;
    J[4][3] = -2*b[0]*q_1+2*b[2]*q_3;
    J[5][0] = 2*b[0]*q_3;
    J[5][1] = 2*b[0]*q_4-4*b[2]*q_2;
    J[5][2] = 2*b[0]*q_1-4*b[2]*q_3;
    J[5][3] = 2*b[0]*q_2;
    norm_df = 0;
    for(int i=0;i<=3;i++){
      Df[i] = J[0][i]*f[0] + J[1][i]*f[1] + J[2][i]*f[2] + J[3][i]*f[3]+ J[4][i]*f[4]+ J[5][i]*f[5];
      norm_df += pow(Df[i],2);
    }
    norm_df = pow(norm_df,.5);

    q_result[0]=q_1-mu*Df[0]/norm_df;
    q_result[1]=q_2-mu*Df[1]/norm_df;
    q_result[2]=q_3-mu*Df[2]/norm_df;
    q_result[3]=q_4-mu*Df[3]/norm_df;

    norm_q = pow(pow(q_result[0],2)+pow(q_result[1],2)+pow(q_result[2],2)+pow(q_result[3],2),.5);

    q_1=q_result[0]/norm_q;
    q_2=q_result[1]/norm_q;
    q_3=q_result[2]/norm_q;
    q_4=q_result[3]/norm_q;
  }
  zk[0] = q_1;
  zk[1] = q_2;
  zk[2] = q_3;
  zk[3] = q_4;

  // Update
  //K = Pk*S
  //S = (Pk+R)^-1
  float det = 0;
  float m[16];
  m[0] = Pk[0][0]+R[0][0];  m[1] = Pk[0][1]+R[0][1];  m[2] = Pk[0][2]+R[0][2];  m[3] = Pk[0][3]+R[0][3];
  m[4] = Pk[1][0]+R[1][0];  m[5] = Pk[1][1]+R[1][1];  m[6] = Pk[1][2]+R[1][2];  m[7] = Pk[1][3]+R[1][3];
  m[8] = Pk[2][0]+R[2][0];  m[9] = Pk[2][1]+R[2][1]; m[10] = Pk[2][2]+R[2][2]; m[11] = Pk[2][3]+R[2][3];
  m[12] =Pk[3][0]+R[3][0]; m[13] = Pk[3][1]+R[3][1]; m[14] = Pk[3][2]+R[3][2]; m[15] = Pk[3][3]+R[3][3];
  S[0][0]  = m[5]  * m[10] * m[15] - m[5]  * m[11] * m[14] -  m[9]  * m[6]  * m[15] +  m[9]  * m[7]  * m[14] +  m[13] * m[6]  * m[11] -    m[13] * m[7]  * m[10];
  S[1][0] = -m[4]  * m[10] * m[15] +   m[4]  * m[11] * m[14] +  m[8]  * m[6]  * m[15] -  m[8]  * m[7]  * m[14] -   m[12] * m[6]  * m[11] +   m[12] * m[7]  * m[10];
  S[2][0] = m[4]  * m[9] * m[15] - m[4]  * m[11] * m[13] -  m[8]  * m[5] * m[15] +  m[8]  * m[7] * m[13] +   m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
  S[3][0] = -m[4]  * m[9] * m[14] + m[4]  * m[10] * m[13] +m[8]  * m[5] * m[14] -  m[8]  * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
  S[0][1] = -m[1]  * m[10] * m[15] + m[1]  * m[11] * m[14] +  m[9]  * m[2] * m[15] -  m[9]  * m[3] * m[14] -  m[13] * m[2] * m[11] +  m[13] * m[3] * m[10];
  S[1][1] = m[0]  * m[10] * m[15] -  m[0]  * m[11] * m[14] -  m[8]  * m[2] * m[15] +  m[8]  * m[3] * m[14] +  m[12] * m[2] * m[11] -   m[12] * m[3] * m[10];
  S[2][1] = -m[0]  * m[9] * m[15] +  m[0]  * m[11] * m[13] +   m[8]  * m[1] * m[15] -   m[8]  * m[3] * m[13] -   m[12] * m[1] * m[11] +   m[12] * m[3] * m[9];
  S[3][1] = m[0]  * m[9] * m[14] -  m[0]  * m[10] * m[13] -  m[8]  * m[1] * m[14] +   m[8]  * m[2] * m[13] +  m[12] * m[1] * m[10] -  m[12] * m[2] * m[9];
  S[0][2] = m[1]  * m[6] * m[15] -  m[1]  * m[7] * m[14] -  m[5]  * m[2] * m[15] +  m[5]  * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
  S[1][2] = -m[0]  * m[6] * m[15] +  m[0]  * m[7] * m[14] +  m[4]  * m[2] * m[15] -  m[4]  * m[3] * m[14] -  m[12] * m[2] * m[7] +  m[12] * m[3] * m[6];
  S[2][2] = m[0]  * m[5] * m[15] - m[0]  * m[7] * m[13] -  m[4]  * m[1] * m[15] +  m[4]  * m[3] * m[13] +  m[12] * m[1] * m[7] -  m[12] * m[3] * m[5];
  S[3][2] = -m[0]  * m[5] * m[14] +  m[0]  * m[6] * m[13] +  m[4]  * m[1] * m[14] -  m[4]  * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
  S[0][3]  = -m[1] * m[6] * m[11] +  m[1] * m[7] * m[10] +  m[5] * m[2] * m[11] -  m[5] * m[3] * m[10] - m[9] * m[2] * m[7] +  m[9] * m[3] * m[6];
  S[1][3] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
  S[2][3] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] +   m[4] * m[1] * m[11] -  m[4] * m[3] * m[9] -   m[8] * m[1] * m[7] +  m[8] * m[3] * m[5];
  S[3][3] = m[0] * m[5] * m[10] -  m[0] * m[6] * m[9] -  m[4] * m[1] * m[10] +  m[4] * m[2] * m[9] +  m[8] * m[1] * m[6] -  m[8] * m[2] * m[5];
  det = 1.0 / (m[0] * S[0][0]  + m[1] * S[1][0] + m[2] * S[2][0] + m[3] * S[3][0]);
  for(int j = 0; j<4; j++){
    for(int i =0; i<4; i++){
      S[i][j] *= det;
  }}
 
   


  //K = Pk*S;
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      for(int k = 0; k < 4; k++){
        K[i][j] += Pk[i][k] * S[k][j];
      }
    }
  }
  float x_new[4] = {0,0,0,0};
  float norm_xnew = 0;
  for(int i = 0; i<=3; i++){
    yk[i] = zk[i] - xk[i];
    x_new[i] = xk[i] + K[i][0]*yk[0] + K[i][1]*yk[1] + K[i][2]*yk[2] + K[i][3]*yk[3];
  }
   norm_xnew = pow(pow(x_new[0],2)+pow(x_new[1],2)+pow(x_new[2],2)+pow(x_new[3],2),.5);
   
   float Km[4][4]={{0,0,0,0},{ 0,0,0,0},{ 0,0,0,0} ,{0,0,0,0}};
    for(int i = 0; i<=3; i++){
       for(int j = 0; j<=3; j++){
         Km[i][j] = -K[i][j];
       }
    }
    for(int i = 0; i<=3; i++){
       Km[i][i] = 1+Km[i][i];
    }
    float tmp5[4][4] = {{0,0,0,0},{ 0,0,0,0},{ 0,0,0,0} ,{0,0,0,0}};
    
    for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      for(int k = 0; k < 4; k++){
        tmp5[i][j]  + Km[i][k] * Pk[k][j];
      }
    }
  }

 for(int i = 0; i < 4; i++){
     P1_new[i] = tmp5[0][i];
     P2_new[i] = tmp5[1][i];
     P3_new[i] = tmp5[2][i];
     P4_new[i] = tmp5[3][i];
 }
 
  q_new[0] = x_new[0]/norm_xnew;
  q_new[1] = x_new[1]/norm_xnew;
  q_new[2] = x_new[2]/norm_xnew;
  q_new[3] = x_new[3]/norm_xnew;
}


void Quaternion2Euler(float q[4],float *e)
{
  e[0] = float(atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2])));
  e[1] = float(asin(2*(q[0]*q[2]-q[3]*q[1])));
  e[2] = float(atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3])));
}

*/
