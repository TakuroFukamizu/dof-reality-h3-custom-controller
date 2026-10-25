#pragma once

void motor_force_safe();
bool motor_begin();
void motor_write(int duty_percent, int direction);
int motor_pwm_hz();
int motor_dir_level();
