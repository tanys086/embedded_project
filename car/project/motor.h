#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <math.h>

// Define GPIO pins for Motor 1
#define PWM_PIN1 2     // GP2 for PWM (Motor 1)
#define DIR_PIN1_1 0   // GP0 for direction (Motor 1 forward)
#define DIR_PIN1_2 1   // GP1 for direction (Motor 1 reverse)

// Define GPIO pins for Motor 2
#define PWM_PIN2 10    // GP10 for PWM (Motor 2)
#define DIR_PIN2_1 9   // GP9 for direction (Motor 2 forward)
#define DIR_PIN2_2 8   // GP8 for direction (Motor 2 reverse)

// Define GPIO pins for buttons
#define BUTTON_CW 21   // GP21 for clockwise control
#define BUTTON_CCW 22  // GP22 for counterclockwise control
#define BUTTON_STOP 20 // GP20 for stop control

// Function prototypes
void setup_motor_pin();
void turn_right();
void setup_pwm(uint gpio, float freq, float duty_cycle);

#endif
