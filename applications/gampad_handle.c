#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_hid.h"
#include "usbh_hid_gamepad.h"
#include "cmsis_os2.h"
#include "string.h"
#include "main.h"
#include "usart.h"
#define ATC_THRESHOLD 60

static char A_T_C(int8_t analog_x, int8_t analog_y);


void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
    HAL_UART_Transmit_DMA(&huart2, "recivingmsg\r\n", 12); //debug打印
    extern osMessageQueueId_t moving_ctrl_queueHandle;
    static HID_GAMEPAD_Info_TypeDef last_info;
    static char last_direction_msg = 'I';
    static char last_button = 'R';

    switch(USBH_HID_GetDeviceType(phost)) {
        case 0x00: {/* 手柄数据 */
            HID_GAMEPAD_Info_TypeDef *info = USBH_HID_GetGamepadInfo(phost);
            if(info == NULL) {
                break;
            }
            char direction_msg = ' ';
            if(info->hat & 0x08) {
                direction_msg = (((info->hat & 0x07) + 1) & 0x07) + 0x41;
                osMessageQueuePut(moving_ctrl_queueHandle, &direction_msg, 0, 0);
            } else {
                if(last_info.hat != info->hat) { /* 只在松开时发送一次松开信号 */
                    direction_msg = 'I';
                    last_direction_msg = 'I';
                    osMessageQueuePut(moving_ctrl_queueHandle, &direction_msg, 0, 0);
                } else {
                    direction_msg = A_T_C(info->lx, info->ly);
                    if(direction_msg != 'I') {
                        osMessageQueuePut(moving_ctrl_queueHandle, &direction_msg, 0, 0);
                    } else {
                        if(last_direction_msg != 'I') { /* 只在松开时发送一次松开信号 */
                            osMessageQueuePut(moving_ctrl_queueHandle, &direction_msg, 0, 0);
                        }
                    }
                    last_direction_msg = direction_msg;
                }
            }
            char button = A_T_C(info->rx, info->ry);
			if((last_button != 'I' && button == 'I') || button != 'I') {
				char msg = button - 'A' + 'J';
				osMessageQueuePut(moving_ctrl_queueHandle, &msg, 0, 0);
			}
            last_button = button;
            if(GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_CIRCLE)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "p", 0, 0);
            }
            if(GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_SQUARE)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "l", 0, 0);
            }
            /** 按下时触发 */
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_TRIANGLE) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_TRIANGLE)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "j", 0, 0);
            }

            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_CROSS) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_CROSS)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "n", 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_SELECT) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_SELECT)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "T", 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_START) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_START)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "S", 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_L1) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_L1)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "a", 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_L2) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_L2)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "b", 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_L3) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_L3)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "c", 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_R1) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_R1)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "d", 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_R2) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_R2)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "e", 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_R3) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_R3)) {
                osMessageQueuePut(moving_ctrl_queueHandle, "f", 0, 0);
            }
			
            memcpy(&last_info, info, sizeof(HID_GAMEPAD_Info_TypeDef));
            break;
        }
        default:
            break;
    }
}


static char A_T_C(int8_t analog_x, int8_t analog_y)
{
    char result = ' ';
    if(analog_x < -ATC_THRESHOLD) {
        if(analog_y < -ATC_THRESHOLD) {
            result = 'D';
        } else if(analog_y > ATC_THRESHOLD) {
            result = 'B';
        } else {
            result = 'C';
        }
    } else if(analog_x > ATC_THRESHOLD) {
        if(analog_y < -ATC_THRESHOLD) {
            result = 'F';
        } else if(analog_y > ATC_THRESHOLD) {
            result = 'H';
        } else {
            result = 'G';
        }
    } else {
        if(analog_y < -ATC_THRESHOLD) {
            result = 'E';
        } else if(analog_y > ATC_THRESHOLD) {
            result = 'A';
        } else {
            result = 'I';
        }
    }
    return result;
}

