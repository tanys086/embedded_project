#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "pico/time.h"

#define IR_LINE_SENSOR_PIN 6 // A0 connected to GPIO pin 6
#define THRESHOLD 200        // Threshold for detecting a black line
#define DEBOUNCE_COUNT 5     // Number of consistent readings needed for stable state

void init_line_sensor();
bool line_detected();
bool is_black_line();

#endif
