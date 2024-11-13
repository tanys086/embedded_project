#ifndef BARCODE_H
#define BARCODE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

// Define pins
#define ADC_PIN 26  // Analog pin connected to GP26
#define BTN_PIN 20  // Maker kit button pin

// Define constants
#define INTERVAL_MS 10
#define NUM_READINGS 20
#define CHANGE_THRESHOLD 1000   // Define threshold for detecting change
#define MAX_RECORDS 28         // Define maximum number of records
#define BINARY_LENGTH 27       // Length of binary array (ignores the first entry)
#define TOTAL_CHAR 44
#define CHAR_SEGMENTS 3        // Number of segments to divide 27 bits (3 * 9 bits)

// Character and binary mappings
char array_char[TOTAL_CHAR] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 'G',
                               'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                               'Y', 'Z', '_', '.', '$', '/', '+', '%', ' ', '*'};

char *array_code[TOTAL_CHAR] = {"000110100", "100100001", "001100001", "101100000", "000110001", "100110000", "001110000",
                                "000100101", "100100100", "001100100", "100001001", "001001001", "101001000", "000011001",
                                "100011000", "001011000", "000001101", "100001100", "001001100", "000011100", "100000011",
                                "001000011", "101000010", "000010011", "100010010", "001010010", "000000111", "100000110",
                                "001000110", "000010110", "110000001", "011000001", "111000000", "010010001", "110010000",
                                "011010000", "010000101", "110000100", "010101000", "010100010", "010001010", "000101010",
                                "011000100", "010010100"};

// Initialise array used to store the reversed binary representation of each character
char *array_reverse_code[TOTAL_CHAR] = {"001011000", "100001001", "100001100", "000001101", "100011000", "000011001",
                                        "000011100", "101001000", "001001001", "001001100", "100100001", "100100100",
                                        "000100101", "100110000", "000110001", "000110100", "101100000", "001100001",
                                        "001100100", "001110000", "110000001", "110000100", "010000101", "110010000",
                                        "010010001", "010010100", "111000000", "011000001", "011000100", "011010000",
                                        "100000011", "100000110", "000000111", "100010010", "000010011", "000010110",
                                        "101000010", "001000011", "000101010", "010001010", "010100010", "010101000",
                                        "001000110","001010010"};

// Variables to store last ADC value and timing information
volatile uint16_t last_adc_value = 0;
volatile bool recording_enabled = true;  // Flag to control recording
volatile uint64_t last_change_time = 0;
uint64_t recorded_times[MAX_RECORDS];
int binary_array[BINARY_LENGTH] = {0};   // Binary array of 27 elements
volatile int record_count = 0;

// Function prototypes
void setup_adc();
bool repeating_timer_callback(repeating_timer_t *rt);
void reset_recording();
void process_timings();
void convert_binary_to_chars(char *result_chars);