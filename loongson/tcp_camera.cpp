#include "tcp_camera.h"


uint8 image_copy[UVC_HEIGHT][UVC_WIDTH];

void tcp_camera_init(void)
{
    // 初始化TCP客户端,需要先打开TCP服务器,这才不会卡主。
    // 初始化TCP客户端,需要先打开TCP服务器,这才不会卡主。
    // 初始化TCP客户端,需要先打开TCP服务器,这才不会卡主。
    /*if(ipaddress == 1)
    {
        char SERVER_IP[] = SERVER_IP_1;
        if(tcp_client_init(SERVER_IP, PORT) == 0)
        {
            printf("tcp_client ok\r\n");
        }
        else
        {
            printf("tcp_client error\r\n");
        
        }

        // 逐飞助手初始化 设置回调函数
         seekfree_assistant_interface_init(tcp_client_send_data, tcp_client_read_data);
    }
    else
    {
        char SERVER_IP[] = SERVER_IP_2;
        if(tcp_client_init(SERVER_IP, PORT) == 0)
        {
            printf("tcp_client ok\r\n");
        }
        else
        {
            printf("tcp_client error\r\n");
        
        }

        // 逐飞助手初始化 设置回调函数
         seekfree_assistant_interface_init(tcp_client_send_data, tcp_client_read_data);
    }

#if(0 == INCLUDE_BOUNDARY_TYPE)
    // 发送摄像头图像图像信息(仅包含原始图像信息)
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], UVC_WIDTH, UVC_HEIGHT);

#elif(1 == INCLUDE_BOUNDARY_TYPE)
    //以后可作为发送二值化后图像的部分
#elif(2 == INCLUDE_BOUNDARY_TYPE)
    //......
#endif

*/

}

void tcp_show_camera(void) 
{
  // 阻塞式等待，图像刷新
        if(wait_image_refresh() < 0)
        {
            // 摄像头未采集到图像，这里需要关闭电机，关闭电调等。
            exit(0);
        }

        memcpy(image_copy[0], rgay_image, UVC_WIDTH * UVC_HEIGHT);

        seekfree_assistant_camera_send();
}
