
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include <math.h>
#include "motor.h"


void setup_motor_pin(){

    // Initialize GPIO pins for direction control for Motor 1
    gpio_init(DIR_PIN1_1);
    gpio_init(DIR_PIN1_2);
    gpio_set_dir(DIR_PIN1_1, GPIO_OUT);
    gpio_set_dir(DIR_PIN1_2, GPIO_OUT);

    // Initialize GPIO pins for direction control for Motor 2
    gpio_init(DIR_PIN2_1);
    gpio_init(DIR_PIN2_2);
    gpio_set_dir(DIR_PIN2_1, GPIO_OUT);
    gpio_set_dir(DIR_PIN2_2, GPIO_OUT);

    // Initialize GPIO pins for button inputs
    gpio_init(BUTTON_CW);
    gpio_init(BUTTON_CCW);
    gpio_init(BUTTON_STOP);
    gpio_set_dir(BUTTON_CW, GPIO_IN);
    gpio_set_dir(BUTTON_CCW, GPIO_IN);
    gpio_set_dir(BUTTON_STOP, GPIO_IN);
    gpio_pull_up(BUTTON_CW);    // Enable pull-up resistor
    gpio_pull_up(BUTTON_CCW);   // Enable pull-up resistor
    gpio_pull_up(BUTTON_STOP);  // Enable pull-up resistor

}

void turn_right() {

    // Start turning right by moving only the left motor forward
    gpio_put(DIR_PIN1_1, 1);  // Motor 1 forward
    gpio_put(DIR_PIN1_2, 0);
    gpio_put(DIR_PIN2_1, 0);  // Motor 2 stationary
    gpio_put(DIR_PIN2_2, 1);
    setup_pwm(PWM_PIN1, 100.0f, 0.52f);  // Set speed for Motor 1
    setup_pwm(PWM_PIN2, 100.0f, 0.54f);  // Set speed for Motor 2 to 0 (stationary)

}