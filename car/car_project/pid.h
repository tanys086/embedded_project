
#ifndef PID_H
#define PID_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <math.h>

// PID constants
extern float Kp;  // Proportional gain
extern float Ki;  // Integral gain
extern float Kd;  // Derivative gain


// Function to compute the control signal
float compute_pid(float setpoint, float current_value, float *integral, float *prev_error);

// Function to set motor duty cycle (to be implemented with actual hardware code)
void set_motor_duty_cycle(float *duty_cycle, float control_signal);

#endif