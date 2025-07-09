#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "zf_common_headfile.h"

#define MOTOR1_DIR   "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM   "/dev/zf_device_pwm_motor_1"

#define MOTOR2_DIR   "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM   "/dev/zf_device_pwm_motor_2"


extern struct pwm_info motor_1_pwm_info;
extern struct pwm_info motor_2_pwm_info;


// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX    (motor_1_pwm_info.duty_max)       
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。 
#define MOTOR2_PWM_DUTY_MAX    (motor_2_pwm_info.duty_max)        

#define MAX_DUTY        (30 )   // 最大 MAX_DUTY% 占空比






// 定义驱动路劲，该路劲由设备树生成
extern struct pwm_info servo_pwm_info;
#define SERVO_MOTOR1_PWM            "/dev/zf_device_pwm_servo"

// 定义主板上舵机频率  请务必注意范围 50-300
// 如果要修改，需要直接修改设备树。
#define SERVO_MOTOR_FREQ            (servo_pwm_info.freq)                       

// 在设备树中，默认设置的10000。如果要修改，需要直接修改设备树。
#define PWM_DUTY_MAX                (servo_pwm_info.duty_max)      

// 定义主板上舵机活动范围 角度                                                     
#define SERVO_MOTOR_L_MAX           (75 )                       
#define SERVO_MOTOR_R_MAX           (105)                       
#define SERVO_DIR                   (1)
// ------------------ 舵机占空比计算方式 ------------------
// 
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
// 
// 那么不同频率下的占空比计算方式就是
// PWM_DUTY_MAX/(1000/freq)*(1+Angle/180) 在 50hz 时就是 PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
// 
// 那么 100hz 下 90度的打角 即高电平时间1.5ms 计算套用为
// PWM_DUTY_MAX/(1000/100)*(1+90/180) = PWM_DUTY_MAX/10*1.5
// 
// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))
extern float servo_motor_duty_middle;
float servocontrol(float offset_use);
void motor_control(float s_angle);
void motor_control_dif(float s_angle, float speed_target);
#endif