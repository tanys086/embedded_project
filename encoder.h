#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include <stdint.h>

// Pin definitions
#define LEFT_ENCODER_PIN 4   // GPIO pin for the left rotary encoder signal
#define RIGHT_ENCODER_PIN 5  // GPIO pin for the right rotary encoder signal

// Constants for distance calculation
#define WHEEL_CIRCUMFERENCE_CM 20.42  // Circumference of the wheel in centimeters
#define NOTCHES_PER_ROTATION 20    // Number of notches in the encoder wheel
#define WHEEL_BASE_CM 13 // Distance between the two wheels in centimeters

extern volatile uint32_t left_pulse_count;  // Declaration only
extern volatile uint32_t right_pulse_count; // Declaration only

// Interrupt service routines for the encoders
void init_encoders();
float calculate_distance_traveled(uint32_t pulse_count);
float calculate_speed(uint32_t pulse_count, uint32_t time_elapsed_ms);
void shared_encoder_isr(uint gpio, uint32_t events);

#endif