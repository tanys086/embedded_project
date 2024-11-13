//NEW MAIN, FREE RTOS IMPLEMENTATION

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "pid.h"
#include "line_sensor.h"
#include "motor.h"

// Variables
float last_left_distance_traveled = 0.0f;
float last_right_distance_traveled = 0.0f;
float left_speed = 0.0f;
float right_speed = 0.0f;
uint32_t last_time = 0;
uint32_t last_left_pulse_count = 0;
uint32_t last_right_pulse_count = 0;
float target_speed = 32.0;
float left_motor_duty = 0.54;
float right_motor_duty = 0.5;
bool motor1_running = false;
bool motor2_running = false;
bool cw_mode = false;
bool cw_mode2 = false;

// Task Handles
TaskHandle_t MotorControlTaskHandle;
TaskHandle_t SensorTaskHandle;
TaskHandle_t ButtonTaskHandle;

// Function prototypes
void MotorControlTask(void *pvParameters);
void SensorTask(void *pvParameters);
void ButtonTask(void *pvParameters);

int main() {
    // Initialize hardware
    stdio_init_all();
    init_ultrasonic();
    init_encoders();
    setup_motor_pin();
    init_line_sensor();
    setup_pwm(PWM_PIN1, 100.0f, 0.52f);
    setup_pwm(PWM_PIN2, 100.0f, 0.54f);

    // Create tasks
    xTaskCreate(MotorControlTask, "Motor Control", 1024, NULL, 2, &MotorControlTaskHandle);
    xTaskCreate(SensorTask, "Sensor", 1024, NULL, 2, &SensorTaskHandle);
    xTaskCreate(ButtonTask, "Button", 1024, NULL, 1, &ButtonTaskHandle);

    // Start the scheduler
    vTaskStartScheduler();

    while (1) {
        // Main loop should be empty as tasks handle everything
    }
    return 0;
}

void MotorControlTask(void *pvParameters) {
    float left_integral = 0.0f;
    float left_prev_error = 0.0f;
    float right_integral = 0.0f;
    float right_prev_error = 0.0f;

    while (1) {
        if (motor1_running && motor2_running) {
            uint32_t current_time = time_us_64() / 1000;
            uint32_t elapsed_time = current_time - last_time;

            left_speed = calculate_speed(left_pulse_count - last_left_pulse_count, elapsed_time);
            right_speed = calculate_speed(right_pulse_count - last_right_pulse_count, elapsed_time);

            float left_control_signal = compute_pid(target_speed, left_speed, &left_integral, &left_prev_error);
            float right_control_signal = compute_pid(target_speed, right_speed, &right_integral, &right_prev_error);

            set_motor_duty_cycle(&left_motor_duty, left_control_signal);
            set_motor_duty_cycle(&right_motor_duty, right_control_signal);

            setup_pwm(PWM_PIN2, 100.0f, left_motor_duty);
            setup_pwm(PWM_PIN1, 100.0f, right_motor_duty);

            last_time = current_time;
            last_left_pulse_count = left_pulse_count;
            last_right_pulse_count = right_pulse_count;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void SensorTask(void *pvParameters) {
    while (1) {
        float distance = measure_distance();
        if (distance <= 12.0) {
            // Stop both motors
            gpio_put(DIR_PIN1_1, 0);
            gpio_put(DIR_PIN1_2, 0);
            gpio_put(DIR_PIN2_1, 0);
            gpio_put(DIR_PIN2_2, 0);
            motor1_running = motor2_running = false;
            vTaskDelay(pdMS_TO_TICKS(1000));

            // Turn right (90 degrees)
            motor1_running = motor2_running = true;
            turn_right();
            vTaskDelay(pdMS_TO_TICKS(815));

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

                vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to prevent CPU overuse
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // Adjust delay based on sensor polling frequency
    }
}


void ButtonTask(void *pvParameters) {
    while (1) {
        if (!gpio_get(BUTTON_STOP)) {
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_put(DIR_PIN1_1, 0);
            gpio_put(DIR_PIN1_2, 0);
            gpio_put(DIR_PIN2_1, 0);
            gpio_put(DIR_PIN2_2, 0);
            motor1_running = motor2_running = false;
        } else if (!gpio_get(BUTTON_CW)) {
            vTaskDelay(pdMS_TO_TICKS(200));
            if (!motor1_running || !cw_mode) {
                gpio_put(DIR_PIN1_1, 1);
                gpio_put(DIR_PIN1_2, 0);
                motor1_running = true;
                cw_mode = true;
            }
            if (!motor2_running || !cw_mode2) {
                gpio_put(DIR_PIN2_1, 1);
                gpio_put(DIR_PIN2_2, 0);
                motor2_running = true;
                cw_mode2 = true;
            }
        } else if (!gpio_get(BUTTON_CCW)) {
            vTaskDelay(pdMS_TO_TICKS(200));
            if (!motor1_running || cw_mode) {
                gpio_put(DIR_PIN1_1, 0);
                gpio_put(DIR_PIN1_2, 1);
                motor1_running = true;
                cw_mode = false;
            }
            if (!motor2_running || cw_mode2) {
                gpio_put(DIR_PIN2_1, 0);
                gpio_put(DIR_PIN2_2, 1);
                motor2_running = true;
                cw_mode2 = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
