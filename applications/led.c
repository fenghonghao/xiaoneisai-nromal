/*
 * @author: bsb
 * @brief: 板载led灯闪烁
*/

#include "main.h"
#include "FreeRTOS.h"

void led_task(void *argument)
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
        vTaskDelay(1000);
    }
}