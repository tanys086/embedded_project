
#include "encoder.h"

// Global variables to store the pulse count for each encoder
volatile uint32_t left_pulse_count = 0;
volatile uint32_t right_pulse_count = 0;


// Interrupt service routines for the encoders
void shared_encoder_isr(uint gpio, uint32_t events) {
    if (gpio == LEFT_ENCODER_PIN) {
        left_pulse_count++;
    } else if (gpio == RIGHT_ENCODER_PIN) {
        right_pulse_count++;
    }
}

void init_encoders(){
    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER_PIN);  // Enable an internal pull-up resistor
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &shared_encoder_isr);

    gpio_init(RIGHT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER_PIN);  // Enable an internal pull-up resistor
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &shared_encoder_isr);
}

// Function to calculate the distance traveled by each wheel based on the pulse count
float calculate_distance_traveled(uint32_t pulse_count) {
    return pulse_count * (WHEEL_CIRCUMFERENCE_CM / NOTCHES_PER_ROTATION);
}

float calculate_speed(uint32_t pulse_count, uint32_t time_elapsed_ms) {
    // Convert time to seconds
    float time_elapsed_sec = time_elapsed_ms / 1000.0f;

    // Calculate distance traveled
    float distance_traveled = calculate_distance_traveled(pulse_count);

    // Calculate speed: Distance / Time
    return distance_traveled / time_elapsed_sec;
}
