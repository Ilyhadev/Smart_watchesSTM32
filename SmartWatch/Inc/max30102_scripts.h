#ifndef MAX30102_SCRIPTS_H
#define MAX30102_SCRIPTS_H

#include "max30102_for_stm32_hal.h"

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>

#define BUFFER_SIZE 100
#define FILTER_SIZE 5
#define MIN_PEAK_HEIGHT 50
#define MIN_PEAK_DISTANCE 200
#define SIGNAL_THRESHOLD 30
#define BPM_AVERAGE_SIZE 10

// MAX30102 object
extern max30102_t max30102;

// Signal processing variables
extern uint32_t ir_buffer[BUFFER_SIZE];
extern int buffer_index;
extern uint32_t filtered_buffer[FILTER_SIZE];
extern int filter_index;


// Peak detection variables
extern uint32_t last_peak_time;
extern float current_bpm;
extern int32_t prev_signal;
extern int32_t prev_prev_signal;
extern bool peak_found;

// BPM averaging variables
extern float bpm_array[BPM_AVERAGE_SIZE];
extern int bpm_counter;
extern float average_bpm;
extern bool array_filled;

// Startup counter to fill buffer
extern uint32_t startup_samples;

// Filtering with moving average
uint32_t apply_moving_average(uint32_t new_sample, uint32_t *filter_buffer, int *filter_index);

// Function to calculate BPM average
float calculate_bpm_average(float *bpm_array, int count);

void configure_MAX30102 (void);

float* calc_curr_avg_bpm (void);

float* render_bpm (void);
# endif
