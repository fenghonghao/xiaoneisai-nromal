#include "qmi8658a.h"

bool qmi8658a_read_reg(uint8_t reg, uint8_t *data, uint16_t size)
{
  return HAL_I2C_Mem_Read(&hi2c2, QMI8658A_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, 0xFF) == HAL_OK;
}
//寄存器写入操作
bool qmi8658a_write_reg(uint8_t reg, uint8_t *data, uint16_t size)
{
  return HAL_I2C_Mem_Write(&hi2c2, QMI8658A_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, size, 1000) == HAL_OK;
}
//直接写数字
bool qmi8658a_write_reg_t(uint8_t reg, uint8_t data)
{
  return HAL_I2C_Mem_Write(&hi2c2, QMI8658A_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000) == HAL_OK;
}

void qmi8658_on_demand_cali(void)
{
//	printf("qmi8658_on_demand_cali start\n");
	osDelay(2200);	// delay 2000ms above
  qmi8658a_write_reg_t(QMI8658A_RESET_ADDR, 0xb0);
	osDelay(10);	// delay
	qmi8658a_write_reg_t(CTRL9, 0xa2);
	osDelay(2200);	// delay 2000ms above
	qmi8658a_write_reg_t(CTRL9, 0);
	osDelay(100);	// delay
//	printf("qmi8658_on_demand_cali done\n");
}

bool init_qmi8658a(void)
{
  uint8_t who_am_i;

  qmi8658_on_demand_cali();
  // 检查设备ID
  if(!qmi8658a_read_reg(WHO_AM_I, &who_am_i, 1)) return false;

  if(who_am_i != 0x05) return false;

  
  uint8_t ctrl1 = 0x00;
  ctrl1 |= (1<<6); // 地址自动递增
  ctrl1 |= (1<<5); // 数据字节模式：0-小端模式; 1-大端模式(默认)
  ctrl1 |= INT1_ENABLE; // 使能INT1
  ctrl1 |= INT2_ENABLE; // 使能INT2
  qmi8658a_write_reg(CTRL1, &ctrl1, 1);

  //加速度配置
  uint8_t ctrl2 = 0x00;
  ctrl2 |= (ACCEL_FS_2g<<4); // ±2g
  ctrl2 |= (ACCEL_ODR_1000hz); // 1000Hz
  qmi8658a_write_reg(CTRL2, &ctrl2, 1);

  //陀螺仪配置
  uint8_t ctrl3 = 0x00;
  ctrl3 |= (GYRO_FS_16dps<<4); // ±16dps
  ctrl3 |= (GYRO_ODR_1000hz); // 1000Hz
  qmi8658a_write_reg(CTRL3, &ctrl3, 1);

  //滤波模式设置
  uint8_t ctrl5 = 0x00;
  qmi8658a_write_reg(CTRL5, &ctrl5, 1);

  uint8_t ctrl7 = 0x00;
  ctrl7 |= 1; // 使能加速度
  ctrl7 |= 1<<1; // 使能陀螺仪
  qmi8658a_write_reg(CTRL7, &ctrl7, 1);

  uint8_t ctrl8 = 0x00;// 还没有配置，0xc0


  return true;
}
