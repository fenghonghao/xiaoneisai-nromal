/*
 * @author: bsb
 * @breif: 蓝牙串口回调
 */
#include "main.h"
#include "usart.h"

uint8_t receiveBLEData[64];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart2)
    {
        HAL_UART_Transmit_DMA(&huart2, receiveBLEData, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receiveBLEData, sizeof(receiveBLEData));
    }
}