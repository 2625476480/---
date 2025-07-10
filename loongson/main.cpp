#include "zf_common_headfile.h"
#include "main.h"



void uvc_camera_read_thd_entry()
{
    struct sched_param param;
	param.sched_priority = 9;
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    while(1)
    {
        pthread_mutex_lock(&offset_mutex);
        run_state = run_temp;
        pthread_mutex_unlock(&offset_mutex);
        if(run_state == 6)
        {
            printf("testsleep\n");
            system_delay_ms(10);
        }
        else
        {
            if(wait_image_refresh() < 0)
            {
                // 摄像头未采集到图像，这里需要关闭电机，关闭电调等。
                shutoff();
                exit(0);
            }

        
            Camera_All_Deal();
            pthread_mutex_lock(&offset_mutex);
            offset_temp = offset;
            run_temp = run_state;
            garage_temp = road_type.Garage;
            pthread_mutex_unlock(&offset_mutex);
        
            // printf("62_r_search_line = %d.\r\n", r_search_flag_62);
            // printf("61_r_search_line = %d.\r\n", r_search_flag_61);
            // printf("60_r_search_line = %d.\r\n", r_search_flag_60);
            // printf("59_r_search_line = %d.\r\n", r_search_flag_59);
            // printf("58_r_search_line = %d.\r\n", r_search_flag_58);
            // printf("57_r_search_line = %d.\r\n", r_search_flag_57);

            if(ips_image_flag)
            {
                ips200_show_gray_image(0, 0, (const uint8 *)image_01, UVC_WIDTH, UVC_HEIGHT);
                //ips200_show_gray_image(0, 0, rgay_image, UVC_WIDTH, UVC_HEIGHT);
            }
            printf("rguai_down = %d.\r\n", right_guai_down);
            printf("rguai_up = %d.\r\n", Right_Up_Find);
            printf("lwcr_y = %d.\r\n", Longest_White_Column_Right[0]);
            printf("lwcr_x = %d.\r\n", Longest_White_Column_Right[1]);
            printf("lwmd =%d.\r\n", mid_longwhite);
            printf("rlose = %d.\r\n", r_lose_value);
            printf("garage = %d.\r\n", road_type.Garage);
            printf("endl = %d.\r\n", encoder_left);
            printf("endr = %d.\r\n", encoder_right);
            printf("length = %f.\r\n", movelength);
            printf("run_state = %d.\r\n", run_state);
        }
    }

}



TimerFdWrapper *timer;

// 定义线程回调函数，用于处理定时器到期事件
void timer_callback(void) 
{
    float offset_use = 0.0f;
    int8 run_state_use = 1;
    uint8 garage_use = 7;
    pthread_mutex_lock(&offset_mutex);
    offset_use = offset_temp;
    run_state_use = run_temp;
    garage_use = garage_temp;
    pthread_mutex_unlock(&offset_mutex);
    
    float servo_angle = 0.0f;
    encoder_left  = encoder_get_count(ENCODER_1);
    encoder_right = -encoder_get_count(ENCODER_2);
    movelength += (encoder_left + encoder_right) * 0.005f;
    speed_real_l = encoder_left;
    speed_real_r = encoder_right;
    speed_real_mid = 0.5 * (speed_real_l + speed_real_r);
    speed_real_dif = speed_real_l - speed_real_r;
    if(run_state_use == 0)
    {
        servo_angle = servocontrol(offset_use);
        motor_control_dif(servo_angle, 160);
    }
    else if(run_state_use == 1)
    {
        if(garage_use == 1)
        {
            servo_angle = servocontrol(offset_use);
            motor_control_dif(servo_angle, 200);

        }
        else if(garage_use == 2)
        {
            servo_angle = servocontrol(offset_use);
            motor_control_dif(servo_angle, 180);
        }
        else if(garage_use == 3)
        {
            servo_angle = servocontrol(offset_use);
            motor_control_dif(servo_angle, 180);
            
        }
        else if(garage_use == 4)
        {
            servo_angle = servocontrol(0);
            motor_control_dif(servo_angle, -50);
        }
        else if(garage_use == 5)
        {
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(76));
            motor_control_dif(10, -120);
        }
        else if(garage_use == 6)
        {
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(90));
            motor_control_dif(0, -120);
        }
        else if(garage_use == 7)
        {
            servo_angle = servocontrol(0);
            motor_control_dif(servo_angle, 180);
        }
        else if(garage_use == 8)
        {
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(76));
            motor_control_dif(10, 200);
        }
        else if(garage_use == 9)
        {
            servo_angle = servocontrol(0);
            motor_control_dif(servo_angle, 50);
        }

    }
    else if(run_state_use== 3)
    {
        if(garage_use == 1)
        {
            servo_angle = servocontrol(offset_use);
            motor_control_dif(servo_angle, 200);
        }
        else if(garage_use == 2)
        {
            servo_angle = servocontrol(offset_use);
            motor_control_dif(servo_angle, 200);

        }
        else if(garage_use == 3)
        {
            servo_angle = servocontrol(offset_use);
            motor_control_dif(servo_angle, 200);
            
        }
        else if(garage_use == 4)
        {
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(104));
            motor_control_dif(-10, 140);
        }
        else if(garage_use == 41)//减速状态
        {
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(104));
            motor_control_dif(-10, -150);
        }
        else if(garage_use == 5)
        {
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(87));
            motor_control_dif(6, -150);
        }
        else if(garage_use == 51)
        {
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(90));
            motor_control_dif(0, -150);
        }
        else if(garage_use == 6)
        {
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(104));
            motor_control_dif(-10, -150);
        }
        else if(garage_use == 7)
        {
            
            motor_control_dif(-10, 50);
        }
        else if(garage_use == 71)
        {
            motor_control_dif(-10, 180);
        }
        else if(garage_use == 8)
        {
            servo_angle = servocontrol(0);
            motor_control_dif(servo_angle, 200);
        }
        else if(garage_use == 9)
        {
            pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(80));
            motor_control_dif(8, 200);
        }

    }
    else if (run_state_use== 2)
    {
        servo_angle = servocontrol(offset_use);
        motor_control_dif(servo_angle, 180);
    }
    else if(run_state_use == 4)
    {
        servo_angle = servocontrol(offset_use);
        motor_control_dif(servo_angle, 280);
    }
    else if(run_state_use == 5) // 惯性停车状态
    {
        printf("88888\n");

    }
    else if(run_state_use == 6)
    {
        printf("flag = %d\r\n", g_car_control.reach_flag);
        imu_callback();

        encoder_callback();

        // pid_inc_cal(&left_pid,  g_car_control.left_speed, g_encoder_left);
        // pid_inc_cal(&right_pid, g_car_control.right_speed, g_encoder_right);
        int16 mid_speed = (g_car_control.left_speed + g_car_control.right_speed) / 2;
        int16 dif_speed = (g_car_control.left_speed - g_car_control.right_speed);

        pid_inc_cal(&mid_motor_pid, mid_speed, g_encoder_mid);
        pid_inc_cal(&dif_motor_pid, dif_speed, g_encoder_dif);
        pwm_control(mid_motor_pid.out_value + dif_motor_pid.out_value, mid_motor_pid.out_value - dif_motor_pid.out_value);
        // pwm_control(left_pid.out_value, right_pid.out_value);

        servo_control(g_car_control.servo_duty);

        if (g_car_control.reach_flag == 999)
        {
            printf("bcdefg\n\n\n");
            pthread_mutex_lock(&offset_mutex);
            run_temp = 0;
            pthread_mutex_unlock(&offset_mutex);
        }
    }
    else if(run_state_use == 7) // 创意状态
    {

    }
    else if(run_state_use == -1) // 结束状态
    {
        motor_control_dif(0, 0);
    }

}

//退出处理
void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    shutoff();
    exit(0);
}
void cleanup()
{
    printf("程序异常退出，执行清理操作\n");
    // 处理程序退出！！！
    // 这里需要关闭电机，关闭电调等。
    shutoff();
}
void shutoff()
{
    pwm_set_duty(MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR2_PWM, 0);
    ips200_clear();
    gpio_set_level(BEEP,0x0);
    big_state = 0;
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint16)SERVO_MOTOR_DUTY(servo_motor_duty_middle));
    close_tcp_send_ladar();
    close_tcp_send_sensor();
    close_tcp_recv_control();
}



uint16 times = 0;
uint16 counts = 0;
//编码器中断

//初始化
void init()
{
    // 注册清理函数
    atexit(cleanup);

    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);
    //read_params();


    // 初始化屏幕
    ips200_init("/dev/fb0");
    pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);

    // 初始化UVC摄像头
    if(uvc_camera_init("/dev/video0", expo) < 0)
    {
        shutoff();
        exit(0);
    }
    big_state = 1;
    ips_image_flag = 1;
    //tcp_image_flag = 1;

    // 获取PWM设备信息
    pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);

    // 创建pid对象
    My_Pid_Init();

    key_busy = false;

    run_state = 7; // 运行状态
    road_type.Garage = 0; // 停车状态
    timer = new TimerFdWrapper(10, timer_callback);
    timer->start();

    // 获取六轴数据和编码器数据，10ms发送一次。
    ThreadWrapper thd_0(tcp_send_sensor_thd_entry);
    // 串口接收雷达数据，通过TCP发送到ubuntu中。
    ThreadWrapper thd_1(tcp_send_ladar_thd_entry);
    // TCP接收ROS2的数据，控制车模运动和转向
    ThreadWrapper thd_2(tcp_recv_control_thd_entry);
    // 读取摄像头线程
    ThreadWrapper thd_3(uvc_camera_read_thd_entry);
    
 
}


//主程序
int main(int argc, char* argv[]) 
{

    // if (argc < 2) {
    //     printf("错误：未提供参数！\n");
    //     return 1;
    // }
    // int num = std::stoi(argv[1]);
    // choose = num;
    init();
    // run_state = num;


    
}