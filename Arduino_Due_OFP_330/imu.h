#define gyroMeasError 3.14159265358979 * (6.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta

#define pi 3.14159265359

void estimate_quaternions(float w_x,float w_y,float w_z,float a_x,float a_y,float a_z,float m_x,float m_y,float m_z,float SEq_1,float SEq_2,float SEq_3,float SEq_4,float w_error_last[3],float dt, float *q_new,float *w_error, int init);
void Quaternion2Euler(float q[4],float *phi,float *theta,float *psi);


