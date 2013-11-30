
#include "Arduino.h"
#include "pins_arduino.h"

void request_IMU_data();
void read_IMU_data(byte *a_bytes,byte *m_bytes,byte *w_bytes);
void unpack_IMU_data(byte *a_bytes,byte *m_bytes,byte *w_bytes, float *a_raw_data, float *m_raw_data,float *w_raw_data);
void cal_IMU_data(float *a_raw_data,float *m_raw_data,float *w_raw_data, float *a_n_xyz, float *m_n_xyz,float *w_dps_xyz);


