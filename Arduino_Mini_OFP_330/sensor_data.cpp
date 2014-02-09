#include <Arduino.h>
#include "calibration.h"

void unpack_IMU_data(byte *a_bytes,byte *m_bytes,byte *w_bytes, float *a_raw_data, float *m_raw_data, float *w_raw_data)
{
  int temp = 0;
  // ACCEL - adxl345
  for (int i=0;i<3;++i) {
    temp = (int)a_bytes[2*i+1];
    if(temp>128) temp-=256;
    a_raw_data[i] = float((int)a_bytes[2*i] + ((temp) << 8));
  }

  //Magnetometer - HMC5843
  for (int i=0;i<3;++i) {
    temp = (int)m_bytes[2*i];
    if(temp>128) temp-=256;
    m_raw_data[i] = float((int)m_bytes[2*i + 1] + ((temp) << 8));
  }

  //Gyro - ITG3200
  for (int i=0;i<3;++i) {
    temp = (int)w_bytes[2*i];
    if(temp>128) temp-=256;
    w_raw_data[i] = float((int)w_bytes[2*i + 1] + ((temp) << 8))/14.375;
  }
}


void cal_IMU_data(float *a_raw_data,float *m_raw_data,float *w_raw_data, float *a_n_xyz, float *m_n_xyz,float *w_dps_xyz)
{
  float a_n_unk[3];
  float m_n_unk[3];
  float w_dps_unk[3];

  for(int i=0;i<3;i++) a_n_unk[i] = a_raw_data[i];
  a_n_unk[0] -= CAL_ACCEL_d_x;
  a_n_unk[0] *= CAL_ACCEL_m_x;
  a_n_unk[1] -= CAL_ACCEL_d_y;
  a_n_unk[1] *= CAL_ACCEL_m_y;
  a_n_unk[2] -= CAL_ACCEL_d_z;
  a_n_unk[2] *= CAL_ACCEL_m_z;

  for(int i=0;i<3;i++) m_n_unk[i] = m_raw_data[i];
  m_n_unk[0] -= CAL_MAG_d_x;
  m_n_unk[0] *= CAL_MAG_m_x;
  m_n_unk[1] -= CAL_MAG_d_y;
  m_n_unk[1] *= CAL_MAG_m_y;
  m_n_unk[2] -= CAL_MAG_d_z;
  m_n_unk[2] *= CAL_MAG_m_z;

  for(int i=0;i<3;i++) w_dps_unk[i] = w_raw_data[i];


  // Axis rotations (x = nose, y = right wing, z = down)
  a_n_xyz[0] = a_n_unk[0];
  a_n_xyz[1] = a_n_unk[1];
  a_n_xyz[2] = -a_n_unk[2];

  m_n_xyz[0] = -m_n_unk[2];
  m_n_xyz[1] = m_n_unk[0];
  m_n_xyz[2] = m_n_unk[1];

  w_dps_xyz[0] = -w_dps_unk[0] * CAL_GYRO_scale;
  w_dps_xyz[1] = -w_dps_unk[1] * CAL_GYRO_scale;
  w_dps_xyz[2] = w_dps_unk[2]  * CAL_GYRO_scale;

  // Calibrate xyz gyro
  w_dps_xyz[0] -= CAL_GYRO_zero_x;
  w_dps_xyz[1] -= CAL_GYRO_zero_y;
  w_dps_xyz[2] -= CAL_GYRO_zero_z;
}



