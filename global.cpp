#include "menu.h"
// 全局变量定义（实际分配内存）
bool key1_flag = false;
bool key2_flag = false;
bool key3_flag = false;
bool key0_flag = false;
bool switch0_flag = false;
bool switch1_flag = false;
bool key_busy = false;

int16 encoder_left = 0;
int16 encoder_right = 0;
float battery_vol = 0;

int8 big_state;
bool ips_image_flag;
bool tcp_image_flag;

float servo_KP = 1.1;
float servo_KD = 3.1;
float servo_out = 0;

int16 expo = 300;
int8 ipaddress = 2;

int8 run_state = 0; 

int16 mid_longwhite = 0; 

float movelength = 0.0f;

float offset_temp = 0.0f;
int8 run_temp = 1;
uint8 garage_temp = 7;

int8 choose = 1;