#include "pid.h"

// PID constants
float Kp = 0.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.0;  // Derivative gain


// Function to compute the control signal
float compute_pid(float setpoint, float current_value, float *integral, float *prev_error) {
    // Compute the error
    float error = setpoint - current_value;  
    
    // Update the integral term (with clamping to prevent wind-up)
    *integral += error;
    if (*integral > 100) *integral = 100;  // Clamping
    if (*integral < -100) *integral = -100;
    
    // Compute the derivative term
    float derivative = error - *prev_error;
    
    // Compute the control signal
    float control_signal = Kp * error + Ki * (*integral) + Kd * derivative;
    
    // Update the previous error
    *prev_error = error;
    
    return control_signal;
}

// Function to set motor duty cycle (to be implemented with actual hardware code)
void set_motor_duty_cycle(float *duty_cycle, float control_signal) {
    *duty_cycle += control_signal;
    if (*duty_cycle > 0.8) *duty_cycle = 0.8;  // Clamp to max duty cycle
    if (*duty_cycle < 0.3) *duty_cycle = 0.3;  // Clamp to min duty cycle
}