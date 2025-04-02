#ifndef QMI8658A_H
#define QMI8658A_H

// #ifdef __cplusplus
// extern "C" {
// #endif


#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include <cmsis_os2.h>
#include <cmsis_os.h>
#include <stdio.h>
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

// QMI8658A寄存器定义
#define QMI8658A_ADDR            0x6A       // I2C地址 (SA0高电平)
#define QMI8658A_RESET_ADDR      0x60       // 复位寄存器地址
#define WHO_AM_I                 0x00       //Device identifier
#define CTRL1                    0x02       //Serial Interface and Sensor Enable
#define CTRL2                    0x03       //Accelerometer Settings
#define CTRL3                    0x04       //Gyroscope Settings
#define CTRL5                    0x06       //Sensor Data Processing Settings
#define CTRL7                    0x08       //Enable Sensors and Configure Data Reads
#define CTRL8                    0x09       //Reserved – Motion Detection Control
#define CTRL9                    0x0A       //Command Register

#define STATUS0                  0x2E
#define AX_L                     0x35
#define AX_H                     0x36
#define AY_L                     0x37
#define AY_H                     0x38
#define AZ_L                     0x39
#define AZ_H                     0x3A
#define GX_L                     0x3B
#define GX_H                     0x3C
#define GY_L                     0x3D
#define GY_H                     0x3E
#define GZ_L                     0x3F
#define GZ_H                     0x40


#define INT1_ENABLE              0x10
#define INT2_ENABLE              0x08

// 加速度配置参数
#define ACCEL_ST                 0x01   // 启用加速度自检,7
//量程设置
#define ACCEL_FS_2g              0x00   // ±2g,6:4
#define ACCEL_FS_4g              0x01   // ±4g
#define ACCEL_FS_8g              0x02   // ±8g
#define ACCEL_FS_16g             0x03   // ±16g
//输出数据率
#define ACCEL_ODR_1000hz         0x03   // 1000Hz,3:0
#define ACCEL_ODR_500hz          0x04   // 500Hz
#define ACCEL_ODR_250hz          0x05   // 250Hz
#define ACCEL_ODR_125hz          0x06   // 125Hz

// 陀螺仪配置参数
#define GYRO_ST                  0x00   // 启用陀螺仪自检,7
#define GYRO_FS_16dps            0x00   // ±16dps,6:4
#define GYRO_FS_32dps            0x01   // ±32dps
#define GYRO_FS_64dps            0x02   // ±64dps
#define GYRO_FS_128dps           0x03   // ±128dps
#define GYRO_FS_256dps           0x04   // ±256dps
#define GYRO_FS_512dps           0x05   // ±512dps
#define GYRO_FS_1024dps          0x06   // ±1024dps
#define GYRO_FS_2048dps          0x07   // ±2048dps
//输出数据率
#define GYRO_ODR_1000hz          0x03   // 896.8Hz,3:0
#define GYRO_ODR_500hz           0x04   // 430.3Hz
#define GYRO_ODR_250hz           0x05   // 215.3Hz
#define GYRO_ODR_125hz           0x06   // 107.6Hz

//滤波模式
#define GYRO_LPF_266             0x00   // 2.66%,6:5
#define GYRO_LPF_336             0x01   // 3.36%
#define GYRO_LPF_539             0x02   // 5.39%
#define GYRO_LPF_1337            0x03   // 13.37%
#define GYRO_LPF_ENABLE          0x01   // 启用陀螺仪低通滤波器,4

#define ACCEL_LPF_266            0x00   // 2.66%,2:1
#define ACCEL_LPF_336            0x01   // 3.36%
#define ACCEL_LPF_539            0x02   // 5.39%
#define ACCEL_LPF_1337            0x03   // 13.37%
#define ACCEL_LPF_ENABLE         0x01   // 启用加速度低通滤波器,0


#define ONE_G                    9.807f
#define PI                       3.14159265358979323846f
#define ssvt_a                   16384U
#define ssvt_g                  2048U
// Function prototypes
extern void qmi8658_on_demand_cali(void);
extern bool init_qmi8658a(void);

//寄存器读取操作
bool qmi8658a_read_reg(uint8_t reg, uint8_t *data, uint16_t size);
//寄存器写入操作
bool qmi8658a_write_reg(uint8_t reg, uint8_t *data, uint16_t size);
//直接写数字
bool qmi8658a_write_reg_t(uint8_t reg, uint8_t data);
// #ifdef __cplusplus
// }
// #endif

#endif // QMI8658A_H