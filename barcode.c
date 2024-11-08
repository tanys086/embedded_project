#include "barcode.h"

// Function to setup the ADC
void setup_adc() {
    adc_init();
    adc_gpio_init(ADC_PIN);  // Initialize ADC pin
    adc_select_input(0);     // Select ADC input (0 corresponds to GP26)
}

// Callback function for the repeating timer
bool repeating_timer_callback(repeating_timer_t *rt) {
    // Check if recording is still enabled
    if (!recording_enabled) return true;

    // Average the ADC readings to reduce noise
    uint32_t adc_sum = 0;
    for (int i = 0; i < NUM_READINGS; i++) {
        adc_sum += adc_read();
    }
    uint16_t avg_adc_value = adc_sum / NUM_READINGS;

    // Detect change based on threshold
    if (abs((int)avg_adc_value - (int)last_adc_value) > CHANGE_THRESHOLD) {
        uint64_t current_time = time_us_64(); // Current time in microseconds

        // Calculate time since last change
        if (record_count > 0) {
            uint64_t time_since_last_change = (current_time - last_change_time) / 1000; // Convert to milliseconds
            recorded_times[record_count] = time_since_last_change;
        } else {
            recorded_times[record_count] = 0;  // For the first entry, we have no previous time
        }

        // Print the recorded time with array index
        printf("[Array Index %d] Recorded Time (ms): %llu\n", record_count, recorded_times[record_count]);

        // Update variables and increment record count
        last_change_time = current_time;
        last_adc_value = avg_adc_value;
        record_count++;

        // Check if array is full
        if (record_count == MAX_RECORDS) {
            printf("Array Full - Processing Timings:\n");
            process_timings();
            printf("\n");

            // Stop further recording
            recording_enabled = false;
        }
    }

    return true;
}

// Process timings to create the binary array
void process_timings() {
    // Find the 9 longest timings (ignoring the first element [0])
    uint64_t longest_times[9] = {0};  // Array to store the top 9 longest times
    int longest_indices[9] = {0};     // Array to store indices of the top 9 longest times

    // Traverse from index 1 to 27 and find the 9 longest timings
    for (int i = 1; i < MAX_RECORDS; i++) {
        uint64_t current_time = recorded_times[i];

        // Check if current time is larger than the smallest in the longest_times array
        for (int j = 0; j < 9; j++) {
            if (current_time > longest_times[j]) {
                // Shift the array to make room for the new time
                for (int k = 8; k > j; k--) {
                    longest_times[k] = longest_times[k - 1];
                    longest_indices[k] = longest_indices[k - 1];
                }
                // Insert the new time and its index
                longest_times[j] = current_time;
                longest_indices[j] = i - 1;  // Store the index offset by 1 for binary array
                break;
            }
        }
    }

    // Populate the binary array based on the longest_indices
    for (int i = 0; i < 9; i++) {
        binary_array[longest_indices[i]] = 1;
    }

    // Print the binary array
    printf("Binary Array:\n");
    for (int i = 0; i < BINARY_LENGTH; i++) {
        printf("%d", binary_array[i]);
    }
    printf("\n");

    // Convert binary array to characters
    char result_chars[CHAR_SEGMENTS + 1] = {0};  // +1 for null terminator
    convert_binary_to_chars(result_chars);
    if (strlen(result_chars) == 0) {
        printf("Invalid Barcode\n");
    } else {
        printf("Converted Characters: %s\n", result_chars);
    }
}

// Convert the binary array into three characters, with reverse code fallback
void convert_binary_to_chars(char *result_chars) {
    for (int segment = 0; segment < CHAR_SEGMENTS; segment++) {
        // Extract 9-bit segment from the binary array
        char binary_segment[10] = {0};  // 9 bits + 1 for null terminator
        for (int i = 0; i < 9; i++) {
            binary_segment[i] = binary_array[segment * 9 + i] ? '1' : '0';
        }

        // Try to find a matching character in array_code
        int matched = 0;
        for (int j = 0; j < TOTAL_CHAR; j++) {
            if (strcmp(binary_segment, array_code[j]) == 0) {
                result_chars[segment] = array_char[j];
                matched = 1;
                break;
            }
        }

        // If no match, try array_reverse_code
        if (!matched) {
            for (int j = 0; j < TOTAL_CHAR; j++) {
                if (strcmp(binary_segment, array_reverse_code[j]) == 0) {
                    result_chars[segment] = array_char[j];
                    matched = 1;
                    break;
                }
            }
        }

        // If no match found in both codes, set result_chars to empty and exit
        if (!matched) {
            result_chars[0] = '\0';  // Empty the result to indicate invalid barcode
            return;
        }
    }
}

// Function to reset the recording array and continue recording
void reset_recording() {
    printf("Resetting recording...\n");
    record_count = 0;
    recording_enabled = true;
    last_change_time = 0;

    // Clear recorded times array
    for (int i = 0; i < MAX_RECORDS; i++) {
        recorded_times[i] = 0;
    }

    // Reset the binary array
    for (int i = 0; i < BINARY_LENGTH; i++) {
        binary_array[i] = 0;
    }
}
