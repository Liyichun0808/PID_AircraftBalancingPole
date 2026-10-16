// MyCode/mpu6050.h
#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define MPU6050_ADDR         0x68
#define MPU6050_ADDR_WRITE   0xD0
#define MPU6050_ADDR_READ    0xD1

#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_WHO_AM_I     0x75

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp_raw;
} MPU6050_RawData;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temperature;
} MPU6050_Data;

uint8_t MPU6050_Init(void);
uint8_t MPU6050_ReadRaw(MPU6050_RawData *raw);
uint8_t MPU6050_ReadAll(MPU6050_Data *data);
uint8_t MPU6050_Calibrate(uint16_t samples);
float MPU6050_GetAngleX(void);
float MPU6050_UpdateAngleX(const MPU6050_RawData *raw);
float MPU6050_GetGyroX(void);
float MPU6050_GetGyroXFromRaw(const MPU6050_RawData *raw);

#endif
