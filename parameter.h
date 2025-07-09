////本文件存储所有可选的显示的参数
#ifndef _PARAMETER_H
#define _PARAMETER_H
extern float battery_vol;
extern int16 encoder_left;
extern int16 encoder_right;

extern int8 big_state;
extern bool ips_image_flag;
extern bool tcp_image_flag;

extern float servo_KP, servo_KD;
extern float servo_out;

extern int16 expo;

extern int8 ipaddress;

extern int8 run_state;

extern int16 mid_longwhite;

extern float movelength;

extern float offset_temp;
extern int8 run_temp;
extern uint8 garage_temp;

extern float ofs_last;

extern int8 choose;
//事实上大部分别的参数都已经在各自文件定义好，整合到一起实在困难，因此不在此处列出。
/*

参数列表：
电池电压    battery_vol
左编码器    encoder_left
右编码器    encoder_right
左目标速度  
右目标速度
电机kp
电机ki
电机kd*
电机out限幅
舵机kp
舵机kd
舵机ki*
舵机限幅
角度环偏差  float offset
图像阈值    uint8 Threshold
左右丢线数  uint8 l_lose_value, r_lose_value
边线长度    uint8 length_l_line, length_r_line
出界标志    uint8 flag_stop


*/
#endif