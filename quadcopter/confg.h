// Pid Roll PID parametreleri
#define kp_roll 130
#define kd_roll 18.0
#define ki_roll 0.04

#define ki_max_roll 400

float setpoint_roll=0.0;
float pid_i_roll_toplamhata;
float pid_sonhata_roll;

float pid_output_roll;

// Pid Pitch PID Parametreleri
#define kp_pitch 130
#define kd_pitch 18.0
#define ki_pitch 0.04

#define ki_max_pitch 400 

float setpoint_pitch=0.0;
float pid_i_pitch_toplamhata;
float pid_sonhata_pitch;

float pid_output_pitch;

float pid_error_temp;


//Motor
float m1,m2,m3,m4;

//Gyro Değerleri
float imu_acc_x,imu_acc_y,imu_acc_z;

//Zamanlar
unsigned loop_timer,timer_channel_1,timer_channel_2,timer_channel_3,timer_channel_4,esc_loop_timer;
