% 轮子速度控制分析
clear;
clc;

% 1. 读取数据
data = readtable('robot_data.csv');
time = data.time;
target_speed = data.pid_output;      % PID输出就是目标速度
actual_speed_L = data.left_motor;    % 左轮实际速度
actual_speed_R = data.right_motor;   % 右轮实际速度

% 2. 绘制速度对比图
figure('Name', '轮子速度控制分析');
plot(time, target_speed, 'k--', 'LineWidth', 1.5);  % 目标速度
hold on;
plot(time, actual_speed_L, 'b-', 'LineWidth', 1.2); % 左轮实际速度
plot(time, actual_speed_R, 'r-', 'LineWidth', 1.2); % 右轮实际速度
title('Wheel Speed Control');
xlabel('Time (s)');
ylabel('Speed (rad/s)');
legend('Target Speed', 'Left Wheel', 'Right Wheel');
grid on;

% 3. 计算控制误差
left_error = mean(abs(actual_speed_L - target_speed));
right_error = mean(abs(actual_speed_R - target_speed));

fprintf('速度控制性能分析:\n');
fprintf('左轮平均跟踪误差: %.4f rad/s\n', left_error);
fprintf('右轮平均跟踪误差: %.4f rad/s\n', right_error);