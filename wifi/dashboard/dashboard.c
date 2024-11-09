#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "pico/time.h"
#include <string.h>
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/netif.h"

#define UART_ID uart1
#define BAUD_RATE 115200

#define UART_TX_PIN 8
#define UART_RX_PIN 9
#define BUTTON_PIN 22

#define TCP_PORT 1234

struct tcp_pcb *persistent_pcb = NULL;

static char previous_turn_adjustment[128] = "";
static char previous_speed_adjustment[128] = "";
static char previous_no_movement[128] = "";

// UART Setup
// void UARTsetup() {
//     uart_init(UART_ID, BAUD_RATE);
//     gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
//     gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
// }

// // UART Connection/loopback testing task
// void loopbackUART_task(void *pvParameters) {
//     while (1) {
//         const char *message = "Loopback Test: Hello UART!\n";
//         uart_puts(UART_ID, message);
//         sleep_ms(1000); // Allow time for output to be read

//         // Read back data from UART1 RX pin (in loopback)
//         char received[100];
//         int i = 0;

//         while (uart_is_readable(UART_ID) && i < sizeof(received) - 1) {
//             received[i++] = uart_getc(UART_ID);  // Read each byte
//         }

//         received[i] = '\0';  // Null-terminate the string

//         // Print received message to confirm loopback
//         printf("Received: %s\n", received);
//         vTaskDelay(pdMS_TO_TICKS(1000)); // FreeRTOS delay
//     }
// }

// Task 2: WiFi Data Task

// void send_data_to_server(struct tcp_pcb *tpcb, const char *data) {
//     err_t write_err = tcp_write(tpcb, data, strlen(data), TCP_WRITE_FLAG_COPY);
//     if (write_err != ERR_OK) {
//         printf("Failed to send data: %d\n", write_err);
//     } else {
//         printf("Data sent to Dashboard: %s\n", data);
//     }

//     err_t output_err = tcp_output(tpcb);
//     if (output_err != ERR_OK) {
//         printf("Failed to flush data: %d\n", output_err);
//     } else {
//         printf("Data flushed successfully\n");
//     }
// }

// Function to handle received data
static err_t tcp_data_received(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        printf("Connection closed by client\n");
        tcp_close(tpcb);
        persistent_pcb = NULL;
        return ERR_OK;
    }

    // Ensure the data is null-terminated
    char buffer[1024];
    memset(buffer, 0, sizeof(buffer));
    memcpy(buffer, p->payload, p->len);
    buffer[p->len] = '\0';  // Null-terminate the string

    // Extract lines from the received data
    char *line = strtok(buffer, "\n");
    bool turn_adjustment_changed = false;
    bool speed_adjustment_changed = false;
    bool rapid_stop_detected = false;
    bool no_movement_detected = false;

    while (line != NULL) {
        // Check if the line is a turn adjustment
        if (strstr(line, "T:")) {
            if (strcmp(line, previous_turn_adjustment) != 0) {
                strncpy(previous_turn_adjustment, line, sizeof(previous_turn_adjustment) - 1);
                previous_turn_adjustment[sizeof(previous_turn_adjustment) - 1] = '\0';
                turn_adjustment_changed = true;
                memset(previous_no_movement, 0, sizeof(previous_no_movement));
            }
        }

        // Check if the line is a speed adjustment
        if (strstr(line, "M:")) {
            if (strcmp(line, previous_speed_adjustment) != 0) {
                strncpy(previous_speed_adjustment, line, sizeof(previous_speed_adjustment) - 1);
                previous_speed_adjustment[sizeof(previous_speed_adjustment) - 1] = '\0';
                speed_adjustment_changed = true;
                memset(previous_no_movement, 0, sizeof(previous_no_movement));
            }
        }

        // Check if the line is a rapid stop message
        if (strstr(line, "RS:")) {
            rapid_stop_detected = true;
            memset(previous_no_movement, 0, sizeof(previous_no_movement));
        }

        // Check if the line is a no movement message
        if (strstr(line, "No movement detected")) {
            if (strcmp(line, previous_no_movement) != 0) {
                strncpy(previous_no_movement, line, sizeof(previous_no_movement) - 1);
                previous_no_movement[sizeof(previous_no_movement) - 1] = '\0';
                no_movement_detected = true;
            } 
        }

        line = strtok(NULL, "\n");
    }

    // Print the buffer if there are changes, rapid stop or no movement is detected
    if (turn_adjustment_changed || speed_adjustment_changed || rapid_stop_detected || no_movement_detected) {
        printf("Data received from client:\n%s\n", buffer);
    }

    tcp_recved(tpcb, p->len);  // Acknowledge that the data has been received
    pbuf_free(p);  // Free the buffer
    
    return ERR_OK;
}

// // Function to handle connection established
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) {
        printf("Connection failed: %d\n", err);
        tcp_close(tpcb);
        persistent_pcb = NULL;
        return err;
    }

    printf("Connection established\n");

    // Set up the receive callback for incoming data
    tcp_recv(tpcb, tcp_data_received);

    persistent_pcb = tpcb;  // Store the persistent connection

    return ERR_OK;
}

void tcp_server_task(void *pvParameters) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("Failed to create PCB\n");
        vTaskDelete(NULL); // Terminate this task
    }

    if (tcp_bind(pcb, IP_ADDR_ANY, TCP_PORT) != ERR_OK) {
        printf("Failed to bind PCB\n");
        tcp_close(pcb);
        vTaskDelete(NULL); // Terminate this task
    }

    pcb = tcp_listen(pcb);
    tcp_accept(pcb, tcp_client_connected);

    printf("TCP server listening on port %d\n", TCP_PORT);

    while (1) {
        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(400)); // FreeRTOS delay
    }
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

        // Print the Pico W's IP address
        const ip4_addr_t *ip = netif_ip4_addr(netif_default);
        if (ip) {
            printf("Pico W IP Address: %s\n", ip4addr_ntoa(ip));
        } else {
            printf("Failed to get IP address\n");
        }
    }

    // Create TCP server task
    xTaskCreate(tcp_server_task, "TCPServer", 256, NULL, 1, NULL);

    while (1) {
        cyw43_arch_poll();
        vTaskDelay(pdMS_TO_TICKS(400)); // FreeRTOS delay
    }

    cyw43_arch_deinit();
}

int main() {
    stdio_init_all();  // Initialize serial I/O
    // UARTsetup();       // Set up UART

    // Create FreeRTOS tasks
    // xTaskCreate(loopbackUART_task, "UART", 256, NULL, 2, NULL); // Higher priority
    xTaskCreate(wifi_connect_task, "WiFi", 256, NULL, 1, NULL); 
    // xTaskCreate(button_press_task, "ButtonPress", 256, NULL, 1, NULL); 

    vTaskStartScheduler();  // Start the FreeRTOS scheduler

    return 0;
}
