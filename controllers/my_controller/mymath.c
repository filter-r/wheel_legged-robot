#include "mymath.h"

Discreteness Theta_LW;
Discreteness Theta_RW;

// 初始化差分结构体
Discreteness create_discreteness(double dt) {
    Discreteness disc;
    disc.last_value = 0.0;
    disc.last_diff = 0.0;
    disc.value = 0.0;
    disc.diff = 0.0;
    disc.dt = dt;
    return disc;
}
// Diff方法实现
double Diff(Discreteness *disc, double current_value) {
    disc->value = current_value;
    disc->diff = (disc->value - disc->last_value) / disc->dt;
    disc->last_value = disc->value;
    disc->last_diff = disc->diff;
    return disc->diff;
}