#include "control.h"
#include <cmath>
float servo_motor_duty_middle = 94;
struct pwm_info servo_pwm_info;
struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;
#define         LUN_JU          0.120f//轮距，左右轮间距
#define         ZHOU_JU         0.155f//轴距，前后轮间距
#define         PI              3.1415926f

#define         OUT_MAX         1500
float ofs_last = 0.0f; // 上次偏移量
float dif_calculate(float servo_angle)
{
    double radius = 0.0f;
    double servo_radians = servo_angle * PI / 180.0f;
    if(func_abs(servo_radians) >= 0.05)
    {
        radius = ZHOU_JU / tan(servo_radians);
        return 0.5 * LUN_JU / radius;
    }
    else
        return 0.0f;
}

float servocontrol(float offset_use)
{
    
    float servo_angle = (servo_KP * offset_use + servo_KD * (offset_use - ofs_last)) * SERVO_DIR;
    // servo_angle = 2;
    servo_out = servo_motor_duty_middle - servo_angle;
    servo_out = servo_out < SERVO_MOTOR_R_MAX ? servo_out : SERVO_MOTOR_R_MAX;
    servo_out = servo_out > SERVO_MOTOR_L_MAX ? servo_out : SERVO_MOTOR_L_MAX;
    ofs_last = offset_use;
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(servo_out));
    return servo_angle;
}

void motor_control(float s_angle)
{
    


    float speed_target_l_ = speed_target_l;
    float speed_target_r_ = speed_target_r;

    int16 speed_out_l;
    int16 speed_out_r;  
    
    speed_pwm_l = PidIncCtrl(&speed_pid_l, speed_target_l_ - speed_real_l, 0.01);
    speed_pwm_r = PidIncCtrl(&speed_pid_r, speed_target_r_ - speed_real_r, 0.01);

    if(speed_pwm_l >= 0)
    {
        speed_out_l = speed_pwm_l > OUT_MAX ? OUT_MAX : speed_pwm_l;
        pwm_set_duty(MOTOR1_PWM, speed_out_l);
        gpio_set_level(MOTOR1_DIR, 1);
    }
    else
    {
        speed_out_l = speed_pwm_l < -OUT_MAX ? -OUT_MAX : speed_pwm_l;
        pwm_set_duty(MOTOR1_PWM, -speed_out_l);
        gpio_set_level(MOTOR1_DIR, 0);
    }
    if(speed_pwm_r >= 0)
    {
        speed_out_r = speed_pwm_r > OUT_MAX ? OUT_MAX : speed_pwm_r;
        pwm_set_duty(MOTOR2_PWM, speed_out_r);
        gpio_set_level(MOTOR2_DIR, 1);
    }
    else
    {
        speed_out_r = speed_pwm_r < -OUT_MAX ? -OUT_MAX : speed_pwm_r;
        pwm_set_duty(MOTOR2_PWM, -speed_out_r);
        gpio_set_level(MOTOR2_DIR, 0);
    }
        // //----------------------------------------------------------------------------
        // pwm_set_duty(MOTOR1_PWM, 16 * 100);
        // gpio_set_level(MOTOR1_DIR, 1);
        // pwm_set_duty(MOTOR2_PWM, 16 * 100);
        // gpio_set_level(MOTOR2_DIR, 1);
}

void motor_control_dif(float s_angle, float speed_target)
{
    float speed_dif_p = dif_calculate(s_angle);
    // printf("speed_dif_p = %f.\r\n", speed_dif_p);
    // printf("s_angle = %f.\r\n", s_angle);
    int16 speed_out_l;
    int16 speed_out_r;  
    float speed_target_mid = (speed_target_l + speed_target_r) * 0.5;
    speed_target_mid = speed_target;
    // printf("speed_tg = %f.\r\n", speed_target);
    // printf("speed_rl = %f.\r\n", speed_real_mid);
    speed_pwm_mid = PidIncCtrl(&speed_pid_mid, speed_target_mid - speed_real_mid, 0.01);
    speed_pwm_dif = PidIncCtrl(&speed_pid_dif, speed_target_mid * speed_dif_p - speed_real_dif, 0.01);
    //解释：speed_real_dif是左大右小，左减右；speed_dif_p同样是左大右小，左减右；
    //因此speed_pwm_dif要加到右轮上，形成反馈
    speed_pwm_l = speed_pwm_mid + speed_pwm_dif;
    speed_pwm_r = speed_pwm_mid - speed_pwm_dif;
    if(speed_pwm_l >= 0)
    {
        speed_out_l = speed_pwm_l > OUT_MAX ? OUT_MAX : speed_pwm_l;
        pwm_set_duty(MOTOR1_PWM, speed_out_l);
        gpio_set_level(MOTOR1_DIR, 1);
    }
    else
    {
        speed_out_l = speed_pwm_l < -OUT_MAX ? -OUT_MAX : speed_pwm_l;
        pwm_set_duty(MOTOR1_PWM, -speed_out_l);
        gpio_set_level(MOTOR1_DIR, 0);
    }
    if(speed_pwm_r >= 0)
    {
        speed_out_r = speed_pwm_r > OUT_MAX ? OUT_MAX : speed_pwm_r;
        pwm_set_duty(MOTOR2_PWM, speed_out_r);
        gpio_set_level(MOTOR2_DIR, 1);
    }
    else
    {
        speed_out_r = speed_pwm_r < -OUT_MAX ? -OUT_MAX : speed_pwm_r;
        pwm_set_duty(MOTOR2_PWM, -speed_out_r);
        gpio_set_level(MOTOR2_DIR, 0);
    }
}
