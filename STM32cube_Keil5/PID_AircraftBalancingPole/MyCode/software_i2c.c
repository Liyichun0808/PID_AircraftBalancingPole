// MyCode/software_i2c.c
#include "software_i2c.h"
#include "gpio.h"
#include <stdio.h>

// ============ Proteus 仿真专用延时 ============
static void SW_I2C_Delay(void)
{
    // Proteus 仿真需要更长的延时
    // 如果通信失败，增大这个值
    for(volatile int i = 0; i < 50; i++);
}

// ============ I2C 起始信号 ============
static void SW_I2C_Start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    SW_I2C_Delay();
    SDA_LOW();
    SW_I2C_Delay();
    SCL_LOW();
    SW_I2C_Delay();
}

// ============ I2C 停止信号 ============
static void SW_I2C_Stop(void)
{
    SDA_LOW();
    SCL_HIGH();
    SW_I2C_Delay();
    SDA_HIGH();
    SW_I2C_Delay();
}

// ============ 发送字节 ============
static uint8_t SW_I2C_WriteByte(uint8_t data)
{
    uint8_t ack;
    
    for(uint8_t i = 0; i < 8; i++) {
        if(data & 0x80) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }
        data <<= 1;
        SW_I2C_Delay();
        SCL_HIGH();
        SW_I2C_Delay();
        SCL_LOW();
        SW_I2C_Delay();
    }
    
    // 接收 ACK
    SDA_HIGH();
    SW_I2C_Delay();
    SCL_HIGH();
    SW_I2C_Delay();
    ack = SDA_READ();
    SCL_LOW();
    SW_I2C_Delay();
    
    return (ack == 0) ? 1 : 0;
}

// ============ 读取字节 ============
static uint8_t SW_I2C_ReadByte(uint8_t send_ack)
{
    uint8_t data = 0;
    
    SDA_HIGH();
    
    for(uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        SCL_HIGH();
        SW_I2C_Delay();
        if(SDA_READ()) {
            data |= 0x01;
        }
        SCL_LOW();
        SW_I2C_Delay();
    }
    
    if(send_ack) {
        SDA_LOW();   // ACK
    } else {
        SDA_HIGH();  // NACK
    }
    SW_I2C_Delay();
    SCL_HIGH();
    SW_I2C_Delay();
    SCL_LOW();
    SW_I2C_Delay();
    SDA_HIGH();
    
    return data;
}

// ============ 初始化 ============
void SW_I2C_Init(void)
{
    // GPIO 已在 CubeMX 中配置
    SDA_HIGH();
    SCL_HIGH();
    SW_I2C_Delay();
    
    printf("SW I2C Initialized (Proteus Simulation)\r\n");
}

// ============ 写寄存器 ============
uint8_t SW_I2C_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    SW_I2C_Start();
    
    if(!SW_I2C_WriteByte(dev_addr & 0xFE)) {
        SW_I2C_Stop();
        return 0;
    }
    
    if(!SW_I2C_WriteByte(reg_addr)) {
        SW_I2C_Stop();
        return 0;
    }
    
    for(uint16_t i = 0; i < len; i++) {
        if(!SW_I2C_WriteByte(data[i])) {
            SW_I2C_Stop();
            return 0;
        }
    }
    
    SW_I2C_Stop();
    return 1;
}

// ============ 读寄存器 ============
uint8_t SW_I2C_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    SW_I2C_Start();
    
    if(!SW_I2C_WriteByte(dev_addr & 0xFE)) {
        SW_I2C_Stop();
        return 0;
    }
    
    if(!SW_I2C_WriteByte(reg_addr)) {
        SW_I2C_Stop();
        return 0;
    }
    
    SW_I2C_Start();
    
    if(!SW_I2C_WriteByte(dev_addr | 0x01)) {
        SW_I2C_Stop();
        return 0;
    }
    
    for(uint16_t i = 0; i < len; i++) {
        data[i] = SW_I2C_ReadByte(i < len - 1);
    }
    
    SW_I2C_Stop();
    return 1;
}

// ============ I2C 扫描 ============
void SW_I2C_Scan(void)
{
    uint8_t found = 0;
    
    printf("Scanning I2C bus...\r\n");
    
    for(uint8_t addr = 0x01; addr < 0x7F; addr++) {
        SW_I2C_Start();
        if(SW_I2C_WriteByte(addr << 1)) {
            printf("Found device at 0x%02X\r\n", addr);
            found++;
        }
        SW_I2C_Stop();
        HAL_Delay(10);
    }
    
    printf("Scan complete. Found %d device(s).\r\n", found);
}