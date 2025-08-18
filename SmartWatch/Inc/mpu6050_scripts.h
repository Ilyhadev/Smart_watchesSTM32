#ifndef MPU6050_SCRIPTS_H
#define MPU6050_SCRIPTS_H
#include "mpu6050.h"

extern mpu6050_t mpu6050;
extern I2C_HandleTypeDef hi2c1;

void configure_MPU6050(void);

float* render_accel (void);

#endif
