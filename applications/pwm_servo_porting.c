#include "pwm_servo.h"
#include <stdint.h>
#include "main.h"
#include "tim.h"

PWMServoObjectTypeDef *pwm_servos[4];

static void pwm_servo1_write_pin(uint32_t new_state)
{
    HAL_GPIO_WritePin(PWM_SERVO_1_GPIO_Port, PWM_SERVO_1_Pin, (GPIO_PinState)new_state);
}

static void pwm_servo2_write_pin(uint32_t new_state)
{
    HAL_GPIO_WritePin(PWM_SERVO_2_GPIO_Port, PWM_SERVO_2_Pin, (GPIO_PinState)new_state);
}

static void pwm_servo3_write_pin(uint32_t new_state)
{
    HAL_GPIO_WritePin(PWM_SERVO_3_GPIO_Port, PWM_SERVO_3_Pin, (GPIO_PinState)new_state);
}

static void pwm_servo4_write_pin(uint32_t new_state)
{
    HAL_GPIO_WritePin(PWM_SERVO_4_GPIO_Port, PWM_SERVO_4_Pin, (GPIO_PinState)new_state);
}

void pwm_servos_init(void)
{
    for(int i = 0; i < 4; ++i) {
        static PWMServoObjectTypeDef pwm_servo_objects[4];
        pwm_servos[i] = &pwm_servo_objects[i];
        pwm_servo_object_init(pwm_servos[i]);  // 初始化PWM舵机对象内存
    }
    pwm_servos[0]->write_pin = pwm_servo1_write_pin;
    pwm_servos[1]->write_pin = pwm_servo2_write_pin;
    pwm_servos[2]->write_pin = pwm_servo3_write_pin;
    pwm_servos[3]->write_pin = pwm_servo4_write_pin;

    /*
    * @brief tim3为200Hz,5ms一个计数周期,时分复用法,轮流输出4个舵机pwm
    */
    //计数值设置为0
    __HAL_TIM_SET_COUNTER(&htim13, 0);
    //清除更新标志
    __HAL_TIM_CLEAR_FLAG(&htim13, TIM_FLAG_UPDATE);
    //清除比较通道1标志
    __HAL_TIM_CLEAR_FLAG(&htim13, TIM_FLAG_CC1);
    //开启中断
    __HAL_TIM_ENABLE_IT(&htim13, TIM_IT_UPDATE);
    //开启比较通道1
    __HAL_TIM_ENABLE_IT(&htim13, TIM_IT_CC1);
    //启用定时器13
    __HAL_TIM_ENABLE(&htim13);
}
