#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <math.h>
#include "pico/time.h"

#include "ultrasonic.h"
#include "encoder.h"
#include "pid.h"
#include "line_sensor.h"
#include "motor.h"


// Variables
float last_left_distance_traveled = 0.0f;  // Store last left distance traveled
float last_right_distance_traveled = 0.0f; // Store last right distance traveled
float left_speed = 0.0f;
float right_speed = 0.0f;
uint32_t last_time = 0;  // Last time the distance was measured
uint32_t last_left_pulse_count = 0;  // Last left pulse count
uint32_t last_right_pulse_count = 0;  // Last right pulse count

// PID control variables
float target_speed = 32.0;  // Target speed in cm/s for both motors
float left_motor_speed = 0.0;  // Current left motor speed
float right_motor_speed = 0.0;  // Current right motor speed
float left_motor_duty = 0.54;  // Initial duty cycle for left motor
float right_motor_duty = 0.5;  // Initial duty cycle for right motor
float left_integral = 0.0, right_integral = 0.0;  // Integral terms for PI=
float left_prev_error = 0.0, right_prev_error = 0.0;  // Previous errors for PID

//motor variables
bool motor1_running = false; // To track if Motor 1 is running
bool motor2_running = false; // To track if Motor 2 is running
bool cw_mode = false;  // Track direction of Motor 1
bool cw_mode2 = false; // Track direction of Motor 2


int main() {
    // Initialize serial communication
    stdio_init_all();
    init_ultrasonic();
    init_encoders();
    setup_motor_pin();
    init_line_sensor();


    // Set up PWM on both motors
    setup_pwm(PWM_PIN1, 100.0f, 0.52f);  // 100 Hz frequency, 50% duty cycle (Motor 1)
    setup_pwm(PWM_PIN2, 100.0f, 0.54f);  // 100 Hz frequency, 50% duty cycle (Motor 2)


    // Time tracking for speed calculation
    uint32_t last_time = time_us_64() / 1000;  // In milliseconds
    uint32_t last_left_pulse_count = left_pulse_count;
    uint32_t last_right_pulse_count = right_pulse_count;
    
    // Control motor direction based on button presses
    while (true) {

        if(motor1_running && motor2_running){
            sleep_ms(100);
            uint32_t current_time = time_us_64() / 1000; // Current time in milliseconds
            uint32_t elapsed_time = current_time - last_time;

            // Calculate the speed for each motor
            float left_motor_speed = calculate_speed(left_pulse_count - last_left_pulse_count, elapsed_time); //ENCODER_H
            float right_motor_speed = calculate_speed(right_pulse_count - last_right_pulse_count, elapsed_time); //ENCODER_H

            printf("Left Motor Speed: %.2f cm/s\n", left_motor_speed);
            printf("Right Motor Speed: %.2f cm/s\n", right_motor_speed);

            // // Compute the control signals for each motor
            // float left_control_signal = compute_pid(target_speed, left_motor_speed, &left_integral, &left_prev_error);
            // float right_control_signal = compute_pid(target_speed, right_motor_speed, &right_integral, &right_prev_error);

            // printf("Left Control Signal: %.2f\n", left_control_signal);
            // printf("Right Control Signal: %.2f\n", right_control_signal);

            // // Adjust duty cycles based on control signals
            // set_motor_duty_cycle(&left_motor_duty, left_control_signal);
            // set_motor_duty_cycle(&right_motor_duty, right_control_signal);

            // printf("Left Motor Duty Cycle: %.2f\n", left_motor_duty);
            // printf("Right Motor Duty Cycle: %.2f\n", right_motor_duty);

            // setup_pwm(PWM_PIN2, 100.0f, left_motor_duty);  // Set speed for Left Motor
            // setup_pwm(PWM_PIN1, 100.0f, right_motor_duty);  // Set speed for Right Motor


            // Update the last time and pulse count for the next calculation
            last_time = current_time;
            last_left_pulse_count = left_pulse_count;
            last_right_pulse_count = right_pulse_count;
            
        }

        

        
        float distance = measure_distance();
        // printf("\n--- Measurement Data ---\n");
        // printf("Ultrasonic Distance: %.2f cm\n", distance);

        // If the distance is 10 cm or less, stop both motors.
        if (distance <= 12.0) {
            gpio_put(DIR_PIN1_1, 0);  // Stop Motor 1
            gpio_put(DIR_PIN1_2, 0);
            gpio_put(DIR_PIN2_1, 0);  // Stop Motor 2
            gpio_put(DIR_PIN2_2, 0);
            motor1_running = motor2_running = false;  // Update motor states
            sleep_ms(1000);
            motor1_running = motor2_running = true;
            turn_right();
            sleep_ms(815);
            motor1_running = motor2_running = false;
            // Reset pulse counts
            left_pulse_count = 0;
            right_pulse_count = 0;

            // Move forward until the average distance is 90 cm
            while (true) {
                // Calculate the distance traveled by each wheel
                float left_distance = calculate_distance_traveled(left_pulse_count); //ENCODER_H
                float right_distance = calculate_distance_traveled(right_pulse_count); //ENCODER_H

                // Calculate the average distance traveled
                float average_distance = (left_distance + right_distance) / 2.0f;

                // Check if the average distance is 90 cm or more
                if (average_distance >= 87.0f) {
                    // Stop both motors
                    gpio_put(DIR_PIN1_1, 0);
                    gpio_put(DIR_PIN1_2, 0);
                    gpio_put(DIR_PIN2_1, 0);
                    gpio_put(DIR_PIN2_2, 0);
                    break;
                }

                // Move both motors forward
                gpio_put(DIR_PIN1_1, 0);
                gpio_put(DIR_PIN1_2, 1);
                gpio_put(DIR_PIN2_1, 0);
                gpio_put(DIR_PIN2_2, 1);
                setup_pwm(PWM_PIN1, 100.0f, 0.52f);
                setup_pwm(PWM_PIN2, 100.0f, 0.54f);

                // uint32_t current_time = time_us_64() / 1000; // Current time in milliseconds
                // uint32_t elapsed_time = current_time - last_time;

                // // Calculate the speed for each motor
                // float left_motor_speed = calculate_speed(left_pulse_count - last_left_pulse_count, elapsed_time); //ENCODER_H
                // float right_motor_speed = calculate_speed(right_pulse_count - last_right_pulse_count, elapsed_time); //ENCODER_H

                // printf("Left Motor Speed: %.2f cm/s\n", left_motor_speed);
                // printf("Right Motor Speed: %.2f cm/s\n", right_motor_speed);

                // // Compute the control signals for each motor
                // float left_control_signal = compute_pid(target_speed, left_motor_speed, &left_integral, &left_prev_error);
                // float right_control_signal = compute_pid(target_speed, right_motor_speed, &right_integral, &right_prev_error);

                // printf("Left Control Signal: %.2f\n", left_control_signal);
                // printf("Right Control Signal: %.2f\n", right_control_signal);

                // // Adjust duty cycles based on control signals
                // set_motor_duty_cycle(&left_motor_duty, left_control_signal);
                // set_motor_duty_cycle(&right_motor_duty, right_control_signal);

                // printf("Left Motor Duty Cycle: %.2f\n", left_motor_duty);
                // printf("Right Motor Duty Cycle: %.2f\n", right_motor_duty);

                // setup_pwm(PWM_PIN2, 100.0f, left_motor_duty);  // Set speed for Left Motor
                // setup_pwm(PWM_PIN1, 100.0f, right_motor_duty);  // Set speed for Right Motor


                // // Update the last time and pulse count for the next calculation
                // last_time = current_time;
                // last_left_pulse_count = left_pulse_count;
                // last_right_pulse_count = right_pulse_count;

                sleep_ms(100);  // Small delay to prevent CPU overuse
            }
        }

        
              

      //ALL MOTOR CONTROL BUTTON

        if (!gpio_get(BUTTON_STOP)) {  // If the stop button is pressed
            sleep_ms(200);  // Debounce delay
            printf("Stop button pressed. Stopping both motors.\n");
            gpio_put(DIR_PIN1_1, 0);  // Stop Motor 1
            gpio_put(DIR_PIN1_2, 0);
            gpio_put(DIR_PIN2_1, 0);  // Stop Motor 2
            gpio_put(DIR_PIN2_2, 0);
            motor1_running = motor2_running = false; // Update state
        } 
        else if (!gpio_get(BUTTON_CW)) {  // If clockwise button is pressed
            sleep_ms(200);  // Debounce delay
            printf("Clockwise button pressed.\n");
            if (!motor1_running || !cw_mode) {
                gpio_put(DIR_PIN1_1, 1);  // Motor 1 forward
                gpio_put(DIR_PIN1_2, 0);
                printf("Motor 1 running clockwise.\n");
                motor1_running = true;
                cw_mode = true;
            }
            if (!motor2_running || !cw_mode2) {
                gpio_put(DIR_PIN2_1, 1);  // Motor 2 forward
                gpio_put(DIR_PIN2_2, 0);
                printf("Motor 2 running clockwise.\n");
                motor2_running = true;
                cw_mode2 = true;
            }
        } 
        else if (!gpio_get(BUTTON_CCW)) {  // If counterclockwise button is pressed
            sleep_ms(200);  // Debounce delay
            printf("Counterclockwise button pressed.\n");
            
            if (!motor1_running || cw_mode) {
                gpio_put(DIR_PIN1_1, 0);  // Motor 1 reverse
                gpio_put(DIR_PIN1_2, 1);
                printf("Motor 1 running counterclockwise.\n");
                motor1_running = true;
                cw_mode = false;
            }
            if (!motor2_running || cw_mode2) {
                gpio_put(DIR_PIN2_1, 0);  // Motor 2 reverse
                gpio_put(DIR_PIN2_2, 1);
                printf("Motor 2 running counterclockwise.\n");
                motor2_running = true;
                cw_mode2 = false;
            }
        }

        // Ensure both motors are stopped if not running
        if (!motor1_running) {
            gpio_put(DIR_PIN1_1, 0);  // Ensure Motor 1 is stopped
            gpio_put(DIR_PIN1_2, 0);
        }
        if (!motor2_running) {
            gpio_put(DIR_PIN2_1, 0);  // Ensure Motor 2 is stopped
            gpio_put(DIR_PIN2_2, 0);
        }
        
        sleep_ms(10);  // Small delay to prevent CPU overuse
    }

    return 0;
}
