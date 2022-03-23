int32 DOF_X          = 0
int32 DOF_Y          = 1
int32 DOF_Z          = 2
int32 DOF_ROLL       = 3
int32 DOF_PITCH      = 4
int32 DOF_YAW        = 5
int32 DOF_SURGE      = 6
int32 DOF_SWAY       = 7
int32 DOF_HEAVE      = 8
int32 DOF_ROLL_RATE  = 9
int32 DOF_PITCH_RATE = 10
int32 DOF_YAW_RATE   = 11

string name
int32[] dofs

mvp_control/PIDgains pid_x
mvp_control/PIDgains pid_y
mvp_control/PIDgains pid_z

mvp_control/PIDgains pid_roll
mvp_control/PIDgains pid_pitch
mvp_control/PIDgains pid_yaw

mvp_control/PIDgains pid_surge
mvp_control/PIDgains pid_sway
mvp_control/PIDgains pid_heave

mvp_control/PIDgains pid_roll_rate
mvp_control/PIDgains pid_pitch_rate
mvp_control/PIDgains pid_yaw_rate