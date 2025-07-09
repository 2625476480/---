#include "img_process_00.h"
#include "pid.h"
#include "menu.h"
#include "parameter.h"
#include "tcp_camera.h"
#include "class_thread_wrapper.h"
#include "class_posix_pit.h"

#include "get_encoder.h"
#include "get_radar.h"
#include "get_imu.h"
#include "tcp_send_ladar.h"
#include "tcp_send_sensor.h"
#include "tcp_recv_control.h"

#include "get_config.h"
#include "user_pid.h"
#include "motor_control.h"
#include "control.h"




#define ENCODER_1           "/dev/zf_encoder_1"
#define ENCODER_2           "/dev/zf_encoder_2"

                                // 舵机动作状态

#define  ADC_REG_PATH "/sys/bus/iio/devices/iio:device0/in_voltage7_raw"
#define  ADC_SCALE_PATH "/sys/bus/iio/devices/iio:device0/in_voltage_scale"


uint16 adc_reg = 0;
float adc_scale = 0;




void shutoff(void);

pthread_mutex_t offset_mutex = PTHREAD_MUTEX_INITIALIZER;