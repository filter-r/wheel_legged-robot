#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <stdio.h>
#include <math.h>
#include <webots/gyro.h>
#include "pid.h"
#include "mymath.h"
#include "config.h"

#define TIME_STEP 64


//设备
const char *device_names[] = {
"motor_LF", "motor_RF", "motor_LR", "motor_RR", "motor_LW", "motor_RW",//电机
"ps_LF", "ps_RF", "ps_LR", "ps_RR", "ps_LW", "ps_RW",//位置传感器
"gyro", "imu", //惯性测量单元
};
WbDeviceTag devices[40];


void init_all(void);
void get_gesture_data();
void get_wheel_data(void);

int main(int argc, char **argv) {

  init_all();
  
  while (wb_robot_step(TIME_STEP) != -1) {
    // 姿态信息
        get_gesture_data();
        print_gesture_data(Roll, Pitch, Yaw, gyrox, gyroy, gyroz);
    // 轮速(rad/s)
        get_wheel_data();
        print_wheel_data(dot_theta_LW, dot_theta_RW);
    //pitch环
        pitch_pid_out = PID_Calc(&chassis_pitch_pid, Pitch, MECHANICAL_ZERO);//机械中值0.0222553f
        wb_motor_set_velocity(motor_LW, pitch_pid_out);
        wb_motor_set_velocity(motor_RW, pitch_pid_out);
    //speed环
        target_v = 0;
        average_v = ((dot_theta_LW+dot_theta_RW)/2)/10;
        speed_pid_out = PID_Calc(&chassis_speed_pid, target_v, average_v);
    //roll环

    // 辅助调参
        double current_time = wb_robot_get_time();
        log_data(current_time, Pitch, pitch_pid_out, dot_theta_LW,dot_theta_RW);

  };
  if (data_file != NULL) {
        fclose(data_file);
        data_file = NULL;
 }
  wb_robot_cleanup();
  return 0;
}


























void init_all()
{
  wb_robot_init();
  init_data_logging();
  //获取
  for (int i = 0; i < 14; i++) {
    devices[i] = wb_robot_get_device(device_names[i]);
  }
  //使能
  wb_gyro_enable(devices[12], TIME_STEP);
  wb_inertial_unit_enable(devices[13], TIME_STEP);
  for (int i = 6; i < 12; i++) {
    wb_position_sensor_enable(devices[i], TIME_STEP);
  }
  //电机初始化
  for (int i = 0; i < 6; i++) {
    wb_motor_set_position(devices[i], INFINITY);
    wb_motor_set_velocity(devices[i], 0);
  }
  //差分初始化
  double dt = TIME_STEP / 1000.0;  // 转换为秒
  Theta_LW = create_discreteness(dt);
  Theta_RW = create_discreteness(dt);
  //PID初始化
  PID_Init(&chassis_pitch_pid, 'P', 160.0f, 0.0f, 0.0f, 100.0f, 100.0f);
  PID_Init(&chassis_roll_pid, 'P', 0.0f, 0.0f, 0.0f, 100.0f, 100.0f);
  PID_Init(&chassis_speed_pid, 'P', 0.0f, 0.0f, 0.0f, 100.0f, 100.0f);
}

void get_gesture_data(void)
{
        const double *angles = wb_inertial_unit_get_roll_pitch_yaw(imu);//imu
        Roll = -angles[1];
        Pitch= -angles[0];
        Yaw = -angles[2];
        const double *gyro_values = wb_gyro_get_values(gyro);//陀螺仪
        gyrox = -gyro_values[1]; // Roll
        gyroy = -gyro_values[0]; // Pitch
        gyroz = -gyro_values[2]; // Yaw
}
void get_wheel_data(void)
{
        theta_LW = wb_position_sensor_get_value(ps_LW);
        theta_RW = wb_position_sensor_get_value(ps_RW);
        dot_theta_LW = Diff(&Theta_LW, theta_LW);
        dot_theta_RW = Diff(&Theta_RW, theta_RW);
}



//轮子逆时针速度正
//机械中值-1.54854此时pitch0.0222553f稍微往前倾





