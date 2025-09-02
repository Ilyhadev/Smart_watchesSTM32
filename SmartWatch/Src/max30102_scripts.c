/*
 * max30102_scripts.c
 *
 *  Created on: Aug 18, 2025
 *      Author: ilia1
 */

#ifdef __cplusplus
extern "C"
{
#endif

#include <max30102_scripts.h>
#include "max30102_for_stm32_hal.h"
#include "ssd1306_fonts.h"
#include "ssd1306.h"

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
// Initialize variables
// Signal processing variables
uint32_t ir_buffer[BUFFER_SIZE] = {0};
int buffer_index = 0;
uint32_t filtered_buffer[FILTER_SIZE] = {0};
int filter_index = 0;

// Peak detection variables
uint32_t last_peak_time = 0;
float current_bpm = 0;
int32_t prev_signal = 0;
int32_t prev_prev_signal = 0;
bool peak_found = false;

// BPM averaging variables
float bpm_array[BPM_AVERAGE_SIZE];
int bpm_counter = 0;
float average_bpm = 0.0;
bool array_filled = false;
// Startup counter to fill buffer
uint32_t startup_samples = 0;

// MAX30102 object
max30102_t max30102;

uint32_t apply_moving_average(uint32_t new_sample, uint32_t *filter_buffer, int *filter_index) {
    filter_buffer[*filter_index] = new_sample;
    *filter_index = (*filter_index + 1) % FILTER_SIZE;

    uint64_t sum = 0;
    for(int i = 0; i < FILTER_SIZE; i++) {
        sum += filter_buffer[i];
    }
    return sum / FILTER_SIZE;
}

float calculate_bpm_average(float *bpm_array, int count) {
    if (count == 0) return 0.0;

    float sum = 0.0;
    for (int i = 0; i < count; i++) {
        sum += bpm_array[i];
    }
    return sum / count;
}

void configure_MAX30102 (void) {
	max30102_init(&max30102, &hi2c1);
    max30102_reset(&max30102);
    HAL_Delay(100); // Allow reset to complete
    max30102_clear_fifo(&max30102);

    // FIFO configuration
    max30102_set_fifo_config(&max30102, max30102_smp_ave_4, 1, 15);

    // Sensor settings
    max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
    max30102_set_adc_resolution(&max30102, max30102_adc_4096);
    max30102_set_sampling_rate(&max30102, max30102_sr_100);

    // LED current settings
    max30102_set_led_current_1(&max30102, 10.0);
    max30102_set_led_current_2(&max30102, 10.0);

    // Enter SpO2 mode
    max30102_set_mode(&max30102, max30102_spo2);
    max30102_set_a_full(&max30102, 1);
    max30102_set_ppg_rdy(&max30102, 1);

    // Initialize BPM array
    for (int i = 0; i < BPM_AVERAGE_SIZE; i++) {
    	bpm_array[i] = 0.0;
    }
}

float* calc_curr_avg_bpm (void) {
	static float curr_avg_ir[3] = {0.0, 0.0, 0.0};
	// Read FIFO
	max30102_read_fifo(&max30102);
	uint32_t ir_value = max30102._ir_samples[0];

	// Apply moving average filter
	uint32_t filtered_value = apply_moving_average(ir_value, filtered_buffer, &filter_index);

	// Store in circular buffer for baseline calculation
	ir_buffer[buffer_index++] = filtered_value;
	if (buffer_index >= BUFFER_SIZE) buffer_index = 0;

	/*// Skip processing until buffer is filled
	if (startup_samples < BUFFER_SIZE) {
		startup_samples++;
		HAL_Delay(10);
		continue;
	}*/

	// Calculate baseline as average
	uint64_t sum = 0;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		sum += ir_buffer[i];
	}
	uint32_t baseline = sum / BUFFER_SIZE;

	// AC component (signal minus baseline)
	int32_t ir_ac = (int32_t)filtered_value - (int32_t)baseline;

	uint32_t current_time = HAL_GetTick();

	// 3-point peak detection
	if (ir_ac > prev_signal && prev_signal > prev_prev_signal &&
			ir_ac > SIGNAL_THRESHOLD && !peak_found) {

		// Peak detected at previous point
		if (current_time - last_peak_time > MIN_PEAK_DISTANCE) {

			// Calculate BPM
			if (last_peak_time > 0) {
				uint32_t interval = current_time - last_peak_time;
				current_bpm = 60000.0 / interval;

				// Validate BPM range (40-200 BPM)
				if (current_bpm >= 40 && current_bpm <= 200) {

					// Add to BPM array for averaging
					bpm_array[bpm_counter] = current_bpm;
					bpm_counter++;

					// Check if array is full
					if (bpm_counter >= BPM_AVERAGE_SIZE) {
						bpm_counter = 0;
						array_filled = true;
					}

					// Calculate average BPM
					int count_for_average = array_filled ? BPM_AVERAGE_SIZE : bpm_counter;
					if (count_for_average > 0) {
						average_bpm = calculate_bpm_average(bpm_array, count_for_average);
					}
				}
			}
			last_peak_time = current_time;
			peak_found = true;
		}
	} else if (ir_ac < prev_signal) {
		peak_found = false;
	}

	// Update signal history
	prev_prev_signal = prev_signal;
	prev_signal = ir_ac;
	curr_avg_ir[0] = current_bpm;
	curr_avg_ir[1] = average_bpm;
	curr_avg_ir[2] = ir_value;
	return curr_avg_ir;
}

float* render_bpm (void) {
	float* curr_avg_ir;
	// Potentially dangerous
	curr_avg_ir = calc_curr_avg_bpm();
	// Display on OLED
	ssd1306_Fill(0); // Clear screen
	if (curr_avg_ir[2] < 10000) {
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString("No finger detected!", Font_7x10, White);
	} else {
		// Display current BPM
		if (curr_avg_ir[0] > 0) {
			char bpm_str[32];
			snprintf(bpm_str, sizeof(bpm_str), "BPM: %.0f", curr_avg_ir[0]);
			ssd1306_SetCursor(0, 15);
			ssd1306_WriteString(bpm_str, Font_7x10, White);
		}

		// Display average BPM
		if (curr_avg_ir[1] > 0) {
			char avg_bpm_str[32];
			snprintf(avg_bpm_str, sizeof(avg_bpm_str), "Avg: %.1f", curr_avg_ir[1]);
			ssd1306_SetCursor(0, 30);
			ssd1306_WriteString(avg_bpm_str, Font_7x10, White);
		}
	}

	ssd1306_UpdateScreen();
	return curr_avg_ir;
}
