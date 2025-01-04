#include "config.h"

#ifndef PID_H
#define PID_H

typedef struct {
    char mode;          // 'P'表示位置式PID，'D'表示增量式PID
    float Kp;
    float Ki;
    float Kd;
    float max_out;      // 输出限幅
    float max_iout;     // 积分限幅
    float Dbuf[3];      // 微分项历史值
    float error[3];     // 误差历史值
    float Pout;         // 比例输出
    float Iout;         // 积分输出
    float Dout;         // 微分输出
    float out;          // PID输出
    float fdb;          // 反馈值
    float set;          // 设定值
} PidTypeDef;

extern PidTypeDef chassis_pitch_pid;
extern PidTypeDef chassis_roll_pid;
extern PidTypeDef chassis_speed_pid;
extern double pitch_pid_out;  // pitch环输出
extern double roll_pid_out;   // roll环输出
extern double speed_pid_out;  // 速度环输出
// 函数声明
void PID_Init(PidTypeDef *pid, char mode, float Kp, float Ki, float Kd, float max_out, float max_iout);
float PID_Calc(PidTypeDef *pid, float ref, float set);
void PID_Clear(PidTypeDef *pid);

#endif // PID_H
