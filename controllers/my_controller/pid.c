#include "pid.h"
#include <math.h>

PidTypeDef chassis_pitch_pid;
PidTypeDef chassis_roll_pid;
PidTypeDef chassis_speed_pid;
double pitch_pid_out = 0;  // pitch环输出
double roll_pid_out = 0;   // roll环输出
double speed_pid_out = 0;  // 速度环输出


// 限幅函数
static float limit_max(float value, float max_value) {
    if (value > max_value) {
        return max_value;
    } else if (value < -max_value) {
        return -max_value;
    }
    return value;
}

// PID初始化
void PID_Init(PidTypeDef *pid, char mode, float Kp, float Ki, float Kd, float max_out, float max_iout) {
    pid->mode = mode;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    
    // 清零所有历史数据
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

// PID计算
float PID_Calc(PidTypeDef *pid, float ref, float set) {
    // 更新误差历史
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = set - ref;

    if (pid->mode == 'P') {  // 位置式PID
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Iout = limit_max(pid->Iout, pid->max_iout);
        
        if (isnan(pid->Iout)) {
            pid->Iout = 0;
        }

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = pid->error[0] - pid->error[1];
        pid->Dout = pid->Kd * pid->Dbuf[0];

        pid->out = pid->Pout + pid->Iout + pid->Dout;
        pid->out = limit_max(pid->out, pid->max_out);
    }
    else if (pid->mode == 'D') {  // 增量式PID
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = pid->error[0] - 2.0f * pid->error[1] + pid->error[2];
        pid->Dout = pid->Kd * pid->Dbuf[0];

        pid->out += pid->Pout + pid->Iout + pid->Dout;
        pid->out = limit_max(pid->out, pid->max_out);
    }

    return pid->out;
}

// PID清零
void PID_Clear(PidTypeDef *pid) {
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
