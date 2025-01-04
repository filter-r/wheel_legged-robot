#ifndef CONFIG_H
#define CONFIG_H
#include <stdio.h>


//轮子逆时针速度正
//四个轮子关节角度
/*
wb_motor_set_position(motor_LF, -0.32);
wb_motor_set_position(motor_RF, -0.32);
wb_motor_set_position(motor_LR, 0.55);
wb_motor_set_position(motor_RR, 0.55);
*/







// 机器人硬件参数
#define LEG_THIGH      0.15f    // 大腿长度(m)
#define LEG_CALF       0.15f    // 小腿长度(m)
#define WHEEL_RADIUS   0.05f    // 轮子半径(m)
#define MECHANICAL_ZERO 0.023125f  // 机械中值(rad)0.0222553f--前倒 变大向后倒

#define M_PI 3.14159265358979323846
/****************************************设备********************************************* */
// 电机
#define motor_LF devices[0]
#define motor_RF devices[1]
#define motor_LR devices[2]
#define motor_RR devices[3]
#define motor_LW devices[4]
#define motor_RW devices[5]
// 位置传感器
#define ps_LF devices[6]
#define ps_RF devices[7]
#define ps_LR devices[8]
#define ps_RR devices[9]
#define ps_LW devices[10]
#define ps_RW devices[11]
// 陀螺仪和IMU
#define gyro devices[12]
#define imu  devices[13]

/********************************************************************************************* */
extern FILE *data_file; 
// 姿态角度
extern double Roll;
extern double Pitch;
extern double Yaw;
// 角速度
extern double gyrox;
extern double gyroy;
extern double gyroz;

extern double theta_LW;    // 左轮角度
extern double theta_RW;    // 右轮角度
extern double dot_theta_LW;  // 左轮角速度
extern double dot_theta_RW;  // 右轮角速度

extern double target_v;// 速度
extern double average_v;

// 函数声明
void init_data_logging(void);
void log_data(double time, double pitch, double pid_out, double left_wheel, double right_wheel);
void print_gesture_data(double roll, double pitch, double yaw, double gyrox, double gyroy, double gyroz);
void print_wheel_data(double left_speed, double right_speed);  // 新增函数声明
#endif // CONFIG_H 