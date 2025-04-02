#include "qmi8658a.h"
#include "stdio.h"
#include "string.h"

extern osSemaphoreId_t IMU_data_readyHandle;

bool qmi8658a_read_data(uint16_t *accel, uint16_t *gyro)
{
  // HAL_UART_Transmit_DMA(&huart3, (uint8_t*)"reading...\n", 11);
    uint8_t data[12];

    // 检查数据状态
    uint8_t status;
    if (!qmi8658a_read_reg(STATUS0, &status, 1))return false;

    if(!(status & 0x03))  return false;// 检查加速度和陀螺仪数据是否更新

    // 读取加速度数据
    if (!qmi8658a_read_reg(AX_L, data, 6)) return false;
    accel[0] = (int16_t)(data[1] << 8 | data[0]);
    accel[1] = (int16_t)(data[3] << 8 | data[2]);
    accel[2] = (int16_t)(data[5] << 8 | data[4]);

    // 读取陀螺仪数据
    if (!qmi8658a_read_reg(GX_L, data, 6)) return false;
    gyro[0] = (int16_t)(data[1] << 8 | data[0]);
    gyro[1] = (int16_t)(data[3] << 8 | data[2]);
    gyro[2] = (int16_t)(data[5] << 8 | data[4]);

    return true;
}

void IMU_task(void *argument)
{
  if (init_qmi8658a())
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)"QMI8658A initialized\n", 21);
  else
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)"QMI8658A initialization failed\n", 31);
  uint16_t accel_raw[3], gyro_raw[3];
  float accel_g[3], gyro_dps[3];
  int accel_gint[3], gyro_dpsint[3];

  for (;;)
  {
    osSemaphoreAcquire(IMU_data_readyHandle, osWaitForever);
    // HAL_UART_Transmit_DMA(&huart3, (uint8_t*)"IMU data ready\n", 15);
    if (qmi8658a_read_data(accel_raw, gyro_raw))
    {
            // 转换加速度数据 (±2g)
          accel_g[0] = (float)(((short)(accel_raw[0]))*ONE_G)/ssvt_a;
          accel_g[1] = (float)(((short)(accel_raw[1]))*ONE_G)/ssvt_a;
          accel_g[2] = (float)(((short)(accel_raw[2]))*ONE_G)/ssvt_a;
            // 转换陀螺仪数据 (±16dps)
          gyro_dps[0] = (float)(((short)(gyro_raw[0]))*PI)/(ssvt_g*180);
          gyro_dps[1] = (float)(((short)(gyro_raw[1]))*PI)/(ssvt_g*180);
          gyro_dps[2] = (float)(((short)(gyro_raw[2]))*PI)/(ssvt_g*180);

            char buffer[128];
            accel_gint[0] = (int)(accel_g[0] * 100);
            accel_gint[1] = (int)(accel_g[1] * 100);
            accel_gint[2] = (int)(accel_g[2] * 100);
            gyro_dpsint[0] = (int)(gyro_dps[0] * 100);  
            gyro_dpsint[1] = (int)(gyro_dps[1] * 100);
            gyro_dpsint[2] = (int)(gyro_dps[2] * 100);
            sprintf(buffer, "%d,%d,%d\n", accel_gint[0], accel_gint[1], accel_gint[2]);
            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)buffer, strlen(buffer));
            // vTaskDelay(500);
    }
    vTaskDelay(100);
  }
}