
#include "ultrasonic.h"

// Ultrasonic sensor pins
#define TRIGGER_PIN 26  // Example: GP3 for trigger
#define ECHO_PIN 27     // Example: GP4 for echo
#define TIMEOUT 30000  // Timeout in microseconds for faster response (30 ms)

// Function to initialize the ultrasonic sensor
void init_ultrasonic() {
    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_put(TRIGGER_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}
// Function to send a trigger pulse for the ultrasonic sensor
void send_trigger_pulse() {
    gpio_put(TRIGGER_PIN, 1);  // Send a high pulse
    sleep_us(10);               // 10-microsecond pulse
    gpio_put(TRIGGER_PIN, 0);   // Reset the pin to low
}

// Function to measure the echo time in microseconds
unsigned long measure_echo_time() {
    unsigned long start_time = 0, end_time = 0;
    unsigned long wait_start = time_us_32();

    // Wait for the echo pin to go high or timeout
    while (gpio_get(ECHO_PIN) == 0) {
        if (time_us_32() - wait_start > TIMEOUT) {
            return 0; // Timeout, no echo received
        }
    }
    start_time = time_us_32();

    // Wait for the echo pin to go low or timeout
    while (gpio_get(ECHO_PIN) == 1) {
        if (time_us_32() - start_time > TIMEOUT) {
            return 0; // Timeout, echo too long
        }
    }
    end_time = time_us_32();

    return end_time - start_time; // Return the echo duration in microseconds
}

// Function to calculate the distance in cm using the ultrasonic sensor
float measure_distance() {
    const int num_samples = 5;
    float total_distance = 0;
    int valid_samples = 0;

    for (int i = 0; i < num_samples; i++) {
        send_trigger_pulse();  // Send the trigger pulse
        unsigned long duration = measure_echo_time();

        if (duration != 0) {
            total_distance += (duration * 0.0343) / 2.0;
            valid_samples++;
        }

        sleep_ms(5); 
        // Small delay between samples
    }

    if (valid_samples > 0) {
        return total_distance / valid_samples;
    } else {
        return -1.0; // Return -1 if no valid readings were captured
    }
}