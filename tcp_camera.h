#ifndef _TCPCAMERA_H
#define _TCPCAMERA_H

#include "zf_common_headfile.h"

#define SERVER_IP_1 "192.168.146.39"
#define SERVER_IP_2 "192.168.169.177"
#define PORT 8888


#define INCLUDE_BOUNDARY_TYPE   0 //设置为显示原图像

extern uint8 image_copy[UVC_HEIGHT][UVC_WIDTH];

void tcp_camera_init(void);
void tcp_show_camera(void) ;

#endif