#include "config.h"
#include <stdio.h>


FILE *data_file = NULL;

// 姿态角度
double Roll = 0;
double Pitch = 0;
double Yaw = 0;

// 角速度
double gyrox = 0;
double gyroy = 0;
double gyroz = 0;

double theta_LW = 0;     // 左轮角度
double theta_RW = 0;     // 右轮角度
double dot_theta_LW = 0;   // 左轮角速度
double dot_theta_RW = 0;   // 右轮角速度

// 速度
double target_v = 0;
double average_v = 0;


void init_data_logging(void) {
    data_file = fopen("E:/wheel_legged_project/robot_data.csv", "w");
    if (data_file != NULL) {
        printf("文件已创建在桌面\n");
        fprintf(data_file, "time,pitch,pid_output,left_motor,right_motor\n");
    }
}
//电机数据
void log_data(double time, double pitch, double pid_out, double left_wheel, double right_wheel) {
    if (data_file != NULL) {
        fprintf(data_file, "%.3f,%.3f,%.3f,%.3f,%.3f\n", 
                time, pitch, pid_out, left_wheel, right_wheel);
        fflush(data_file);  
    }
}

void print_gesture_data(double roll, double pitch, double yaw, 
                      double gyrox, double gyroy, double gyroz) {
    printf("------------------------\n");
    printf("IMU - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    printf("陀螺仪角速度数据:\n");
    printf("gyrox: %.2f deg/s (Roll)\n", gyrox);
    printf("gyroy: %.2f deg/s (Pitch)\n", gyroy);
    printf("gyroz: %.2f deg/s (Yaw)\n", gyroz);
    printf("------------------------\n");
}
void print_wheel_data(double left_speed, double right_speed) {
    printf("轮速 - 左轮: %.2f rad/s, 右轮: %.2f rad/s\n", left_speed, right_speed);
}


