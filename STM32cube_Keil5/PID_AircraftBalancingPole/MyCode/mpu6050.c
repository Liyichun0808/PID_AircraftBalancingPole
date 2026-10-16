// MyCode/mpu6050.c
#include "mpu6050.h"
#include "software_i2c.h"
#include <math.h>
#include <stdio.h>

static float gyro_scale = 131.0f;
static float accel_scale = 16384.0f;
static float gyro_x_offset = 0.0f;
static float gyro_y_offset = 0.0f;
static float gyro_z_offset = 0.0f;
static float angle_x = 0.0f;
static uint32_t last_time = 0;

static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    return SW_I2C_Write(MPU6050_ADDR_WRITE, reg, &data, 1);
}

static uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    return SW_I2C_Read(MPU6050_ADDR_READ, reg, data, len);
}

uint8_t MPU6050_Init(void)
{
    uint8_t who_am_i;
    
    printf("MPU6050: Initializing...\r\n");
    
    if (!MPU6050_ReadReg(MPU6050_WHO_AM_I, &who_am_i, 1)) {
        printf("MPU6050: I2C communication failed!\r\n");
        return 0;
    }
    
    printf("MPU6050: WHO_AM_I = 0x%02X\r\n", who_am_i);
    
    if (who_am_i != 0x68) {
        printf("MPU6050: Wrong ID (expected 0x68)\r\n");
        return 0;
    }
    
    /* Wake the device and verify every configuration write. */
    if (!MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00)) {
        printf("MPU6050: write PWR_MGMT_1 failed\r\n");
        return 0;
    }
    HAL_Delay(100);
    
    if (!MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 9)) {
        printf("MPU6050: write SMPLRT_DIV failed\r\n");
        return 0;
    }
    
    if (!MPU6050_WriteReg(MPU6050_CONFIG, 0x03)) {
        printf("MPU6050: write CONFIG failed\r\n");
        return 0;
    }
    
    if (!MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00)) {
        printf("MPU6050: write GYRO_CONFIG failed\r\n");
        return 0;
    }
    
    if (!MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00)) {
        printf("MPU6050: write ACCEL_CONFIG failed\r\n");
        return 0;
    }
    
    last_time = HAL_GetTick();
    printf("MPU6050: Initialization OK!\r\n");
    
    return 1;
}

uint8_t MPU6050_ReadRaw(MPU6050_RawData *raw)
{
    uint8_t buf[14];
    
    if (!MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H, buf, 14)) {
        return 0;
    }
    
    raw->accel_x = (buf[0] << 8) | buf[1];
    raw->accel_y = (buf[2] << 8) | buf[3];
    raw->accel_z = (buf[4] << 8) | buf[5];
    raw->temp_raw = (buf[6] << 8) | buf[7];
    raw->gyro_x = (buf[8] << 8) | buf[9];
    raw->gyro_y = (buf[10] << 8) | buf[11];
    raw->gyro_z = (buf[12] << 8) | buf[13];
    
    return 1;
}

uint8_t MPU6050_ReadAll(MPU6050_Data *data)
{
    MPU6050_RawData raw;
    
    if (!MPU6050_ReadRaw(&raw)) {
        return 0;
    }
    
    data->accel_x = (float)raw.accel_x / accel_scale * 9.80665f;
    data->accel_y = (float)raw.accel_y / accel_scale * 9.80665f;
    data->accel_z = (float)raw.accel_z / accel_scale * 9.80665f;
    
    data->gyro_x = (float)raw.gyro_x / gyro_scale - gyro_x_offset;
    data->gyro_y = (float)raw.gyro_y / gyro_scale - gyro_y_offset;
    data->gyro_z = (float)raw.gyro_z / gyro_scale - gyro_z_offset;
    
    data->temperature = (float)raw.temp_raw / 340.0f + 36.53f;
    
    return 1;
}

void MPU6050_Calibrate(uint16_t samples)
{
    MPU6050_RawData raw;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    uint16_t valid = 0;
    
    printf("Calibrating MPU6050...\r\n");
    
    for(uint16_t i = 0; i < samples; i++) {
        if (MPU6050_ReadRaw(&raw)) {
            gx_sum += (float)raw.gyro_x / gyro_scale;
            gy_sum += (float)raw.gyro_y / gyro_scale;
            gz_sum += (float)raw.gyro_z / gyro_scale;
            valid++;
        }
        HAL_Delay(10);
    }
    
    if (valid > 0) {
        gyro_x_offset = gx_sum / valid;
        gyro_y_offset = gy_sum / valid;
        gyro_z_offset = gz_sum / valid;
        
        printf("Calibration OK: X=%ld, Y=%ld, Z=%ld mdps\r\n",
               (long)(gyro_x_offset * 1000.0f),
               (long)(gyro_y_offset * 1000.0f),
               (long)(gyro_z_offset * 1000.0f));
    }
}

float MPU6050_GetAngleX(void)
{
    MPU6050_RawData raw;

    if (!MPU6050_ReadRaw(&raw)) {
        return angle_x;
    }

    return MPU6050_UpdateAngleX(&raw);
}

float MPU6050_UpdateAngleX(const MPU6050_RawData *raw)
{
    MPU6050_Data data;
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    
    if (dt > 0.1f) dt = 0.01f;
    
    data.accel_x = (float)raw->accel_x / accel_scale * 9.80665f;
    data.accel_y = (float)raw->accel_y / accel_scale * 9.80665f;
    data.accel_z = (float)raw->accel_z / accel_scale * 9.80665f;
    data.gyro_x = (float)raw->gyro_x / gyro_scale - gyro_x_offset;
    
    float accel_angle = atan2f(data.accel_y, data.accel_z) * 180.0f / 3.14159f;
    float alpha = 0.96f;
    angle_x = alpha * (angle_x + data.gyro_x * dt) + (1.0f - alpha) * accel_angle;
    
    last_time = now;
    return angle_x;
}

float MPU6050_GetGyroX(void)
{
    MPU6050_Data data;
    if (!MPU6050_ReadAll(&data)) {
        return 0.0f;
    }
    return data.gyro_x;
}
