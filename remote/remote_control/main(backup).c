#include <stdio.h>
#include <stdlib.h>  // Include for abs() function
#include "FreeRTOS.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/netif.h"
#include "task.h"
#include "queue.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// LSM303DLHC (GY-511) I2C address for accelerometer and magnetometer
#define LSM303_ACC_ADDR  0x19
#define LSM303_MAG_ADDR  0x1E

// Registers for accelerometer
#define CTRL_REG1_A      0x20
#define OUT_X_L_A        0x28

// Registers for magnetometer
#define CRA_REG_M        0x00
#define CRB_REG_M        0x01
#define MR_REG_M         0x02
#define OUT_X_H_M        0x03

// I2C pins
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

// I2C port
#define I2C_PORT i2c0

// Define movement thresholds
#define TILT_THRESHOLD 5000
#define Z_STATIONARY_THRESHOLD 3000 
#define Z_MOVEMENT_THRESHOLD 8000
#define MAX_TILT_THRESHOLD 13500  // Adjust this based on your tilt sensitivity requirements
#define MIN_TILT_THRESHOLD 0
#define NOISE_THRESHOLD 200  // Adjust this threshold based on sensitivity needs
#define RAPID_DECEL_THRESHOLD 25  // Threshold for rapid deceleration (in percentage)
#define RAPID_STOP_THRESHOLD 25 // Threshold for rapid stop when moving backward (in percentage)
#define COOLDOWN_PERIOD_MS 5000  // 2-second cooldown after rapid stop
absolute_time_t last_stop_time;
bool is_in_cooldown = false;
int previous_tilt_y = 0;

// Noise cancellation filter parameters
#define FILTER_SIZE 10  // Size of the moving average filter

// Wifi TCP
#define TCP_PORT 1234
#define SERVER_IP "172.20.10.9"
#define WIFI_SSID "AymChris"
#define WIFI_PASSWORD "skdow1029JQ"

struct tcp_pcb *client_pcb = NULL;

// Buffers for noise cancellation
int16_t accel_x_buffer[FILTER_SIZE], accel_y_buffer[FILTER_SIZE], accel_z_buffer[FILTER_SIZE];
int buffer_index = 0;  // Index to keep track of current buffer position

// Calibration offsets (added for calibration)
int16_t x_offset = 0, y_offset = 0, z_offset = 0;

// Function to handle connection established
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) {
        printf("Connection failed: %d\n", err);
        tcp_close(tpcb);
        client_pcb = NULL;
        return err;
    }

    // printf("Connection established, sending data...\n");

    client_pcb = tpcb;  // Store the client PCB for future use

    // Example data to send
    char data[] = "Hello from Pico W client!";
    err_t write_err = tcp_write(tpcb, data, strlen(data), TCP_WRITE_FLAG_COPY);
    // if (write_err != ERR_OK) {
    //     printf("Failed to send data: %d\n", write_err);
    // } else {
    //     printf("Data sent to server: %s\n", data);
    // }

    err_t output_err = tcp_output(tpcb);

    return ERR_OK;
}

// Wi-Fi connection task
void wifi_connect_task(void *pvParameters) {
    if (cyw43_arch_init()) {
        printf("Failed to initialize Wi-Fi\n");
        vTaskDelete(NULL); // Terminate this task
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect to Wi-Fi\n");
        cyw43_arch_deinit();
        vTaskDelete(NULL); // Terminate this task
    } else {
        printf("Connected to Wi-Fi\n");
    }

    // Connect to the TCP server
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("Failed to create PCB\n");
        vTaskDelete(NULL); // Terminate this task
    }

    ip_addr_t server_ip;
    ip4addr_aton(SERVER_IP, &server_ip);
    err_t connect_err = tcp_connect(pcb, &server_ip, TCP_PORT, tcp_client_connected);
    if (connect_err != ERR_OK) {
        printf("Failed to initiate connection: %d\n", connect_err);
        tcp_abort(pcb);  // Use tcp_abort to immediately free resources
    } else {
        printf("Connection initiated successfully\n");
    }

    while (1) {
        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(400)); // FreeRTOS delay
    }

    cyw43_arch_deinit();
}

// Function to scan the I2C bus
void i2c_scanner() {
    printf("Scanning I2C bus...\n");
    for (int addr = 1; addr < 127; addr++) {
        int result = i2c_write_blocking(I2C_PORT, addr, NULL, 0, false);
        if (result == PICO_ERROR_GENERIC) {
            continue;  // Address not in use
        }
        printf("I2C device found at address 0x%02x\n", addr);
    }
    printf("Scan complete.\n");
}

// Function to initialize the accelerometer
void lsm303dlhc_init() {
    uint8_t data[] = {CTRL_REG1_A, 0x77};  // 400 Hz data rate, all axes enabled, normal mode
    int result = i2c_write_blocking(I2C_PORT, LSM303_ACC_ADDR, data, 2, false);
    if (result == PICO_ERROR_GENERIC) {
        printf("Error: Failed to write to accelerometer during initialization.\n");
    } else {
        printf("Accelerometer initialized successfully.\n");
    }
}

// Moving average filter functions for noise cancellation
void update_buffer(int16_t *buffer, int16_t new_value) {
    buffer[buffer_index] = new_value;  // Store new value at the current buffer index
}

int16_t get_average(int16_t *buffer) {
    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += buffer[i];
    }
    return sum / FILTER_SIZE;  // Calculate the average
}

// Function to read raw accelerometer data (unmodified)
void read_accelerometer(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    uint8_t buffer[6];
    uint8_t reg = OUT_X_L_A | 0x80;  // Auto-increment flag for reading multiple bytes
    
    // Set the register address to start reading from
    int result = i2c_write_blocking(I2C_PORT, LSM303_ACC_ADDR, &reg, 1, true);
    if (result == PICO_ERROR_GENERIC) {
        printf("Error: Failed to set accelerometer register.\n");
        return;
    }

    // Read accelerometer data (6 bytes)
    result = i2c_read_blocking(I2C_PORT, LSM303_ACC_ADDR, buffer, 6, false);
    if (result == PICO_ERROR_GENERIC) {
        printf("Error: Failed to read accelerometer data.\n");
    } else {
        // Combine high and low bytes for X, Y, Z axes
        *accel_x = (int16_t)((buffer[1] << 8) | buffer[0]);
        *accel_y = (int16_t)((buffer[3] << 8) | buffer[2]);
        *accel_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    }
}

// Calibration function to determine offsets for each axis (added for calibration)
void calibrate_accelerometer() {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int samples = 100;  // Number of samples to average for calibration

    printf("Calibrating accelerometer... Please keep the device stable.\n");

    for (int i = 0; i < samples; i++) {
        int16_t raw_x, raw_y, raw_z;
        read_accelerometer(&raw_x, &raw_y, &raw_z);

        sum_x += raw_x;
        sum_y += raw_y;
        sum_z += raw_z;
        sleep_ms(10);  // Small delay to ensure readings are spaced out
    }

    // Calculate the average for each axis to determine offsets
    x_offset = sum_x / samples;
    y_offset = sum_y / samples;
    z_offset = (sum_z / samples) - 10000;  // Expected resting value for Z in raw units

    printf("Calibration complete: X offset: %d, Y offset: %d, Z offset: %d\n", x_offset, y_offset, z_offset);
}

// Modified function to read and calibrate accelerometer data with noise cancellation
void read_and_calibrate_filter_accelerometer(int16_t *filtered_x, int16_t *filtered_y, int16_t *filtered_z) {
    int16_t raw_x, raw_y, raw_z;
    read_accelerometer(&raw_x, &raw_y, &raw_z);  // Get raw readings

    // Apply calibration offsets to raw readings
    raw_x -= x_offset;
    raw_y -= y_offset;
    raw_z -= z_offset;

    // Update buffers for each axis with calibrated readings
    update_buffer(accel_x_buffer, raw_x);
    update_buffer(accel_y_buffer, raw_y);
    update_buffer(accel_z_buffer, raw_z);

    // Calculate filtered values as the average of the buffer
    *filtered_x = get_average(accel_x_buffer);
    *filtered_y = get_average(accel_y_buffer);
    *filtered_z = get_average(accel_z_buffer);

    // Move to the next buffer position, wrapping around at FILTER_SIZE
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
}

// Function to manage cooldown and dynamic recalibration
void manage_cooldown_and_recalibration() {
    if (is_in_cooldown && absolute_time_diff_us(last_stop_time, get_absolute_time()) >= COOLDOWN_PERIOD_MS * 1000) {
        // printf("Cooldown ended. Recalibrating...\n");
        calibrate_accelerometer();  // Trigger recalibration
        is_in_cooldown = false;
    }
}

void send_data_to_server(const char *data) {
    if (client_pcb) {
        err_t write_err = tcp_write(client_pcb, data, strlen(data), TCP_WRITE_FLAG_COPY);
        // if (write_err != ERR_OK) {
        //     printf("Failed to send data: %d\n", write_err);
        // } else {
        //     printf("Data sent to server: %s\n", data);
        // }

        err_t output_err = tcp_output(client_pcb);
        // if (output_err != ERR_OK) {
        //     printf("Failed to flush data: %d\n", output_err);
        // } else {
        //     printf("Data flushed successfully\n");
        // }
    } else {
        printf("No active TCP connection\n");
    }
}

void check_tilt_direction(int16_t accel_x, int16_t accel_y, int16_t accel_z) {

    manage_cooldown_and_recalibration();

    // Check if cooldown period is active
    if (is_in_cooldown && absolute_time_diff_us(last_stop_time, get_absolute_time()) < COOLDOWN_PERIOD_MS * 1000) {
        send_data_to_server("In cooldown period, ignoring input.\n");
        return;  // Skip the rest of the function if in cooldown
    } else {
        is_in_cooldown = false;  // Reset cooldown flag if cooldown period has ended
    }

    accel_x = -accel_x;
    accel_y = -accel_y;

    if (abs(accel_x) < NOISE_THRESHOLD) accel_x = 0;
    if (abs(accel_y) < NOISE_THRESHOLD) accel_y = 0;

    char buffer[128];
    // snprintf(buffer, sizeof(buffer), "Filtered Accelerometer X: %d, Y: %d, Z: %d\n", accel_x, accel_y, accel_z);
    // send_data_to_server(buffer);

    // Calculate absolute tilt percentages based on X and Y values
    int tilt_x = (abs(accel_x) * 100) / MAX_TILT_THRESHOLD;
    int tilt_y = (abs(accel_y) * 100) / MAX_TILT_THRESHOLD;

    // Clamp tilt values to multiples of 10 up to 100
    tilt_x = (tilt_x / 10) * 10;
    tilt_y = (tilt_y / 10) * 10;
    if (tilt_x > 100) tilt_x = 100;
    if (tilt_y > 100) tilt_y = 100;

    // Reset speed immediately if tilt is below the minimum threshold
    if (tilt_x < MIN_TILT_THRESHOLD) {
        tilt_x = 0;
    }
    if (tilt_y < MIN_TILT_THRESHOLD) {
        tilt_y = 0;
    }

    // Rapid deceleration logic: Stop immediately if a large decrease in tilt_y is detected when moving forward
    if ((previous_tilt_y - tilt_y) >= RAPID_DECEL_THRESHOLD) {
        send_data_to_server("RS:F\n"); // Rapid Stop : Foward
        tilt_y = 0;
        is_in_cooldown = true;  // Activate cooldown
        last_stop_time = get_absolute_time();  // Record the time of the stop
    }
    
    // Rapid stop logic for backward movement: Stop immediately if a large increase in tilt_y is detected
    if ((tilt_y - previous_tilt_y) >= RAPID_STOP_THRESHOLD && accel_y < 0) {
        send_data_to_server("RS:B\n"); // Rapid stop : Backward
        tilt_y = 0;
        is_in_cooldown = true;  // Activate cooldown
        last_stop_time = get_absolute_time();  // Record the time of the stop
    }

    // Update the previous tilt_y for the next comparison
    previous_tilt_y = tilt_y;

    // Determine wheel speed adjustments based on tilt direction and thresholds
    if (tilt_x > 0) {
        if (accel_x >= 0) {
            snprintf(buffer, sizeof(buffer), "T:LW:%d%%\n", tilt_x); // Turn : Left Wheel
            send_data_to_server(buffer);
            // command_left_wheel_speed(tilt_x);
        } else {
            snprintf(buffer, sizeof(buffer), "T:RW:%d%%\n", tilt_x); // Turn : Right Wheel 
            send_data_to_server(buffer);
            // command_right_wheel_speed(tilt_x);
        }
    } else {
        // send_data_to_server("T:NT\n"); // Turn : No Turn
    }

    if (tilt_y > 0) {
        if (accel_y >= 0) {
            snprintf(buffer, sizeof(buffer), "M:MF:%d%%\n", tilt_y); // Move : Moving Foward
            send_data_to_server(buffer);
            // command_both_wheels_speed(tilt_y);
        } else {
            snprintf(buffer, sizeof(buffer), "M:MB:%d%% backwards\n", tilt_y);// Move : Moving Backward
            send_data_to_server(buffer);
            // command_both_wheels_speed(-tilt_y);
        }
    } else {
        // send_data_to_server("M:NM\n"); // Move : No Movement
    }
}

// Accelerometer task
void accelerometer_task(void *pvParameters) {
    // Initialize I2C
    printf("Initializing I2C...\n");
    i2c_init(I2C_PORT, 100 * 1000);  // 100kHz I2C
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C initialized successfully.\n");

    // Initialize the accelerometer
    lsm303dlhc_init();

    // Perform calibration
    calibrate_accelerometer();

    // Variables to store filtered accelerometer readings
    int16_t accel_x, accel_y, accel_z;

    while (1) {
        // Read, calibrate, and filter accelerometer data
        read_and_calibrate_filter_accelerometer(&accel_x, &accel_y, &accel_z);

        // Check tilt direction based on the calibrated and filtered readings
        check_tilt_direction(accel_x, accel_y, accel_z);

        // Delay before the next reading
        vTaskDelay(pdMS_TO_TICKS(400)); // FreeRTOS delay
    }
}

int main() {
    // Initialize serial output (USB/UART)
    stdio_init_all();
    sleep_ms(5000);

    // Create FreeRTOS tasks
    xTaskCreate(wifi_connect_task, "WiFi", 256, NULL, 1, NULL); 
    xTaskCreate(accelerometer_task, "Accelerometer", 256, NULL, 1, NULL); 

    vTaskStartScheduler();  // Start the FreeRTOS scheduler

    return 0;
}
