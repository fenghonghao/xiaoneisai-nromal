/*
 * @author: bsb
 * @breif: 串口接收回调
 */
#include "main.h"
#include "usart.h"
#include "mecanum_chassis.h"
uint8_t receiveBLEData[64];
uint8_t receivePIData[64];
extern ChassisTypeDef *chassis;

bool check(uint8_t data[])
{
    uint8_t index = 0;
    uint8_t checksum = 0;
    while(data[index] != 0xAA && index < 64) index++;
    if(index == 64) return false; //没有找到0xAA
    index ++; //跳过0xAA
    while(data[index + 1] != 0x55 && index < 63)  checksum ^= data[index ++];
    if(index < 63 && data[index + 1] != 0x55) return false; //没有找到0x55
    return checksum == data[index]; //校验和正确返回true
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart2)
    {
        HAL_UART_Transmit_DMA(&huart2, receiveBLEData, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveBLEData, sizeof(receiveBLEData));
    }
    else if(huart == &huart3)
    {
        // HAL_UART_Transmit_DMA(&huart3, receivePIData, Size);
        if(check(receivePIData))
        {
            HAL_UART_Transmit_DMA(&huart3, "check ok\r\n", 12);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, receivePIData, sizeof(receivePIData));
    }
}