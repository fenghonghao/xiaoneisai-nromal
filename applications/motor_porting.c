#include "main.h"
#include "tim.h"
#include "motors_param.h"

/* 全局变量 */
EncoderMotorObjectTypeDef *motors[4];
/* static void packet_handler(struct PacketRawFrame *frame); */

static void motor1_set_pulse(EncoderMotorObjectTypeDef *self, int speed);
static void motor2_set_pulse(EncoderMotorObjectTypeDef *self, int speed);
static void motor3_set_pulse(EncoderMotorObjectTypeDef *self, int speed);
static void motor4_set_pulse(EncoderMotorObjectTypeDef *self, int speed);

void set_motor_param(EncoderMotorObjectTypeDef *motor, int32_t tpc, float rps_limit, float kp, float ki, float kd)
{
    motor->ticks_per_circle = tpc;
    motor->rps_limit = rps_limit;
    motor->pid_controller.kp = kp;
    motor->pid_controller.ki = ki;
    motor->pid_controller.kd = kd;
}

void set_motor_type(EncoderMotorObjectTypeDef *motor, MotorTypeEnum type) {
	switch(type) {
		case MOTOR_TYPE_JGB520:
			set_motor_param(motor, MOTOR_JGB520_TICKS_PER_CIRCLE, MOTOR_JGB520_RPS_LIMIT, MOTOR_JGB520_PID_KP, MOTOR_JGB520_PID_KI, MOTOR_JGB520_PID_KD);
			break;
		case MOTOR_TYPE_JGB37:
			set_motor_param(motor, MOTOR_JGB37_TICKS_PER_CIRCLE, MOTOR_JGB37_RPS_LIMIT, MOTOR_JGB37_PID_KP, MOTOR_JGB37_PID_KI, MOTOR_JGB37_PID_KD);
			break;
		case MOTOR_TYPE_JGA27:
			set_motor_param(motor, MOTOR_JGA27_TICKS_PER_CIRCLE, MOTOR_JGA27_RPS_LIMIT, MOTOR_JGA27_PID_KP, MOTOR_JGA27_PID_KI, MOTOR_JGA27_PID_KD);
			break;
		case MOTOR_TYPE_JGB528:
			set_motor_param(motor, MOTOR_JGB528_TICKS_PER_CIRCLE, MOTOR_JGB528_RPS_LIMIT, MOTOR_JGB528_PID_KP, MOTOR_JGB528_PID_KI, MOTOR_JGB528_PID_KD);
			break;
		default:
			break;
	}
}

void motors_init(void)
{
    for(int i = 0; i < 4; ++i) {
        static EncoderMotorObjectTypeDef motors_objects[4];
        motors[i] = &motors_objects[i];
        encoder_motor_object_init(motors[i]);
		motors[i]->ticks_overflow = 60000;
        motors[i]->ticks_per_circle = MOTOR_DEFAULT_TICKS_PER_CIRCLE;
        motors[i]->rps_limit = MOTOR_DEFAULT_RPS_LIMIT;
        motors[i]->pid_controller.set_point = 0.0f;
        motors[i]->pid_controller.kp = MOTOR_DEFAULT_PID_KP;
        motors[i]->pid_controller.ki = MOTOR_DEFAULT_PID_KI;
        motors[i]->pid_controller.kd = MOTOR_DEFAULT_PID_KD;
    }

    /* 马达 1 */
    motors[0]->set_pulse = motor1_set_pulse;
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* 编码器 */
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim5);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);


    /* 马达 2 */
    motors[1]->set_pulse = motor2_set_pulse;
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* 编码器 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim2);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    /* 马达 3 */
    motors[2]->set_pulse = motor3_set_pulse;
    __HAL_TIM_SET_COUNTER(&htim9, 0);
    __HAL_TIM_ENABLE(&htim9);
    __HAL_TIM_MOE_ENABLE(&htim9);

    /* 编码器 */
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim4);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* 马达 4 */
    motors[3]->set_pulse = motor4_set_pulse;
    __HAL_TIM_SET_COUNTER(&htim10, 0);
    __HAL_TIM_SET_COUNTER(&htim11, 0);
    __HAL_TIM_ENABLE(&htim10);
    __HAL_TIM_ENABLE(&htim11);
    __HAL_TIM_MOE_ENABLE(&htim10);
    __HAL_TIM_MOE_ENABLE(&htim11);

    /* 编码器 */
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim3);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);


    // 测速更新定时器
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim7);

    //packet_register_callback(&packet_controller, PACKET_FUNC_MOTOR, packet_handler);
}

static void motor1_set_pulse(EncoderMotorObjectTypeDef *self, int speed)
{
    if(speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
    } else if(speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -speed);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}


static void motor2_set_pulse(EncoderMotorObjectTypeDef *self, int speed)
{
    if(speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
    } else if(speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

static void motor3_set_pulse(EncoderMotorObjectTypeDef *self, int speed)
{
    HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
    if(speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed);
    } else if(speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, -speed);
    } else {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    }
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}

static void motor4_set_pulse(EncoderMotorObjectTypeDef *self, int speed)
{
    HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
    if(speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, speed);
    } else if(speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, -speed);
    } else {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
    }
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}


// #pragma pack(1)
// typedef struct {
//     uint8_t motor_id;
//     float speed;
// } MotorCtrlCmdUnit;

// typedef struct {
//     uint8_t cmd;
//     uint8_t motor_num;
//     MotorCtrlCmdUnit units[];
// } MotorMutilCtrlCommandTypeDef;

// typedef struct {
//     uint8_t cmd;
//     uint8_t motor_id;
//     float speed;
// } MotorSignalCtrlCommandTypeDef;

// typedef struct {
//     uint8_t cmd;
//     uint8_t motor_id;
// } MotorSignalStopCommandTypedef;

// typedef struct {
//     uint8_t cmd;
//     uint8_t motor_mask;
// } MotorMultiStopCommandTypedef;
// #pragma pack()

/**
* @brief 串口命令回调处理
* @param frame 数据帧
* @retval void
*/
/*
static void packet_handler(struct PacketRawFrame *frame)
{

    switch(frame->data_and_checksum[0]) {
        case 0: {
            MotorSignalCtrlCommandTypeDef *mscc = NULL;
            mscc = (MotorSignalCtrlCommandTypeDef *)frame->data_and_checksum;
            motors[mscc->motor_id]->pid_controller.set_point = mscc->speed;
            break;
        }
        case 1: {
            MotorMutilCtrlCommandTypeDef *mmcc = NULL;
            mmcc = (MotorMutilCtrlCommandTypeDef *)frame->data_and_checksum;
            for(int i = 0; i < mmcc->motor_num; ++i) {
                motors[mmcc->units[i].motor_id]->pid_controller.set_point = mmcc->units[i].speed;
            }
            break;
        }
        case 2:  {
            MotorSignalStopCommandTypedef *mssc = NULL;
            mssc = (MotorSignalStopCommandTypedef *)frame->data_and_checksum;
            motors[mssc->motor_id]->pid_controller.set_point = 0;
            break;
        }
        case 3: {
            MotorMultiStopCommandTypedef *mmsc = NULL;
            mmsc = (MotorMultiStopCommandTypedef *)frame->data_and_checksum;
            for(int i = 0; i < 4; ++i) {
                if(mmsc->motor_mask & (0x01 << i)) {
                    motors[i]->pid_controller.set_point = 0;
                }
            }
            break;
        }
        default:
            break;
    }
}

*/

