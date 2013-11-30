


void control_law(int ThrottleIn, int PitchIn, int RollIn, int YawIn, int FlapIn, float phi, float theta, float psi, float w_dps_xyz[3],float a_n_xyz[3],  int *motor_fr_cmd, int *motor_fl_cmd, int *motor_br_cmd,int *motor_bl_cmd);
void control_law_rate(int ThrottleIn, int PitchIn, int RollIn, int YawIn, int FlapIn, float phi, float theta, float psi, float w_dps_xyz[3],float a_n_xyz[3],  int *motor_fr_cmd, int *motor_fl_cmd, int *motor_br_cmd,int *motor_bl_cmd);
