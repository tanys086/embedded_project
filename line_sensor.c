#include "line_sensor.h"

#define IR_LINE_SENSOR_PIN 6 // A0 connected to GPIO pin 6
#define THRESHOLD 200        // Threshold for detecting a black line
#define DEBOUNCE_COUNT 5     // Number of consistent readings needed for stable state

// Function to initialize the line sensor
void init_line_sensor()
{
    // Initialize the pin as input with a pull-up resistor
    gpio_init(IR_LINE_SENSOR_PIN);
    gpio_set_dir(IR_LINE_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(IR_LINE_SENSOR_PIN);
}

// Function to check if a line is detected
bool line_detected()
{
    if (gpio_get(IR_LINE_SENSOR_PIN) == 1) // Assuming line detection pulls the signal LOW
    {
        printf("LINE DETECTED\n");
        return true;
    }
    printf("NO LINE\n");
    return false;
}

bool is_black_line() {
    static int black_count = 0;   // Counts for stable black readings
    static int white_count = 0;   // Counts for stable white readings
    static bool current_line_status = false;  // Current debounced line status

    uint16_t analog_value = adc_read();

    // Print the raw analog value for debugging purposes
    printf("Analog value: %u\n", analog_value);

    // Detect if the current reading suggests a black line or white surface
    if (analog_value > THRESHOLD) {  // Above 200 indicates black line
        black_count++;
        white_count = 0;  // Reset white count if we get a black reading

        // Only confirm black line if we get `DEBOUNCE_COUNT` consecutive black readings
        if (black_count >= DEBOUNCE_COUNT && !current_line_status) {
            current_line_status = true;
        }
    } else {  // Below 200 indicates white surface
        white_count++;
        black_count = 0;  // Reset black count if we get a white reading

        // Only confirm white surface if we get `DEBOUNCE_COUNT` consecutive white readings
        if (white_count >= DEBOUNCE_COUNT && current_line_status) {
            current_line_status = false;
        }
    }

    return current_line_status;
}