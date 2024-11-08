#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/time.h"

// Ultrasonic sensor pins
#define TRIGGER_PIN 26  // Example: GP3 for trigger
#define ECHO_PIN 27     // Example: GP4 for echo
#define TIMEOUT 30000  // Timeout in microseconds for faster response (30 ms)

void init_ultrasonic();
void send_trigger_pulse();
unsigned long measure_echo_time();
float measure_distance();

#endif