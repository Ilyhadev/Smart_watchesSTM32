/*
 * mpu6050_scripts.c
 *
 *  Created on: Aug 18, 2025
 *      Author: ilia1
 */
#ifdef __cplusplus
extern "C"
{
#endif
// TODO: step counting algorithm
#include "mpu6050.h"
#include "ssd1306_fonts.h"
#include "ssd1306.h"

#include <string.h>
#include <stdio.h>

mpu6050_t mpu6050;

// Function exists to hide driver for main, so it won't get access to inner structure: mpu6050
void configure_MPU6050(void) {
	// hi2c1 is global
	MPU6050_Init(&mpu6050, &hi2c1);
    float offsetXYZ[3] = {188.75, 240, 3361.5}; // enter your values
    float scaleXYZ[3] = {16358.75, 16312.67, 16721.5}; // enter your values
    MPU6050_Set_Accel_Offset_Scale(&mpu6050, offsetXYZ, scaleXYZ);
}

float* render_accel (void) {
	static float accels[3];
	char buf[100];

	MPU6050_Read_Fifo(&mpu6050);
	ssd1306_Fill(0);

	ssd1306_SetCursor (0,0);
	accels[0] = mpu6050.accelerometer.Axyz[0];
	sprintf (buf, "Ax: %.2f ", mpu6050.accelerometer.Axyz[0]);
	ssd1306_WriteString (buf, Font_14x15, White);

	ssd1306_SetCursor (0,20);
	strcpy(buf, "");
	accels[1] = mpu6050.accelerometer.Axyz[1];
	sprintf (buf, "Ay: %.2f ", mpu6050.accelerometer.Axyz[1]);
	ssd1306_WriteString (buf, Font_14x15, White);

	ssd1306_SetCursor (0,40);
	strcpy(buf, "");
	accels[2] = mpu6050.accelerometer.Axyz[2];
	sprintf (buf, "Az: %.2f ", mpu6050.accelerometer.Axyz[2]);
	ssd1306_WriteString (buf, Font_14x15, White);

	ssd1306_UpdateScreen();
	return accels;
}

