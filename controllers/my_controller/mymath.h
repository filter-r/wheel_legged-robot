#ifndef MYMATH_H
#define MYMATH_H

typedef struct {
    double last_value;    // 上一次的值
    double last_diff;     // 上一次的差分值
    double value;         // 当前值
    double diff;          // 当前差分
    double dt;           // 时间步长
} Discreteness;

extern Discreteness Theta_LW;
extern Discreteness Theta_RW;

// 函数声明
Discreteness create_discreteness(double dt);
double Diff(Discreteness *disc, double current_value);

#endif // MYMATH_H 