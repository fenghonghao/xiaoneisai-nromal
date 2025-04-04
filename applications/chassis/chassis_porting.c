#include "mecanum_chassis.h"
#include "encoder_motor.h"
#include "chassis.h"
#include "motors_param.h"

MecanumChassisTypeDef      jetauto;
extern EncoderMotorObjectTypeDef *motors[4];


static void jetank_set_motors(void* self, float rps_l, float rps_r)
{
    encoder_motor_set_speed(motors[0], rps_r);
    encoder_motor_set_speed(motors[3], -rps_l);
}


static void tankblack_set_motors(void* self, float rps_l, float rps_r)
{
    encoder_motor_set_speed(motors[0], -rps_l);
    encoder_motor_set_speed(motors[1], rps_r);
}

static void ti4wd_set_motors(void* self, float rps_l, float rps_r)
{
    encoder_motor_set_speed(motors[3], rps_r);
    encoder_motor_set_speed(motors[2], rps_r);
    encoder_motor_set_speed(motors[0], -rps_l);
    encoder_motor_set_speed(motors[1], -rps_l);
}

static void jetauto_set_motors(void* self, float rps_lh, float rps_lt, float rps_rh, float rps_rt)
{
    encoder_motor_set_speed(motors[3], rps_lh);
    encoder_motor_set_speed(motors[2], rps_lt);
    encoder_motor_set_speed(motors[0], -rps_rh);
    encoder_motor_set_speed(motors[1], -rps_rt);
}

static void jetacker_set_motors(void* self, float rps_lh, float rps_lt,int position)
{   
	static int position_last = 0;
	if( position_last != position )
	{
		serial_servo_set_position(9,position,100);
		position_last = position;
	}
    encoder_motor_set_speed(motors[1], rps_lh);
    encoder_motor_set_speed(motors[0], rps_lt);
}

ChassisTypeDef *chassis = (ChassisTypeDef*)&jetauto;

void chassis_init(void)
{

    mecanum_chassis_object_init(&jetauto);
    jetauto.base.chassis_type = CHASSIS_TYPE_JETAUTO;
    jetauto.correction_factor = JETAUTO_CORRECITION_FACTOR;
    jetauto.wheel_diameter = JETAUTO_WHEEL_DIAMETER;
    jetauto.shaft_length = JETAUTO_SHAFT_LENGTH;
    jetauto.wheelbase = JETAUTO_WHEELBASE;
    jetauto.set_motors = jetauto_set_motors;
		
	// ackermann_chassis_object_init(&jetacker);
    // jetacker.base.chassis_type = CHASSIS_TYPE_JETACKER;
    // jetacker.correction_factor = JETACKER_CORRECITION_FACTOR;
    // jetacker.wheel_diameter = JETACKER_WHEEL_DIAMETER;
    // jetacker.shaft_length = JETACKER_SHAFT_LENGTH;
    // jetacker.wheelbase = JETACKER_WHEELBASE;
    // jetacker.set_motors = jetacker_set_motors;

}

void set_chassis_type(uint8_t chassis_type)
{
    switch(chassis_type) {
        case CHASSIS_TYPE_JETAUTO:
            chassis = (ChassisTypeDef*)&jetauto;
            set_motor_type(motors[0], MOTOR_TYPE_JGB520);
            set_motor_type(motors[1], MOTOR_TYPE_JGB520);
            set_motor_type(motors[2], MOTOR_TYPE_JGB520);
            set_motor_type(motors[3], MOTOR_TYPE_JGB520);
            break;
        default:
            break;
    }
}

