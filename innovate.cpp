#include "innovate.h"
#include "get_config.h"

innovate_typedef innovate_recv;
innovate_typedef innovate_use;
static int innovate_recv_socket;

void close_innovate_recv(void)
{
    close(innovate_recv_socket);
}

void innovate_recv_thd_entry(void)
{
    // 尝试设置新的高优先级,值越大优先级越高（1-99）
	struct sched_param param;
	param.sched_priority = 6;
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    printf("tcp_init recv ing... \r\n");

    while(1)
    {
        do
        {
            // 初始化 tcp server
            innovate_recv_socket = tcp_init(SERVER_IP, INNOVATE_PORT);
            if(innovate_recv_socket < 0)
            {
                system_delay_ms(1000);
                printf("innovate_recv_socket error \r\n");
            }
            else
            {
                printf("innovate_recv_socket OK \r\n");
            }
        } while (innovate_recv_socket <= 0);

        while(1)
        {
            // TCP周期发送传感器数据
            int str_len = tcp_client_read_data(innovate_recv_socket, (uint8_t *)&innovate_recv, sizeof(innovate_typedef));
            
            if(str_len <= 0)
            {
                innovate_recv.choose = 0;
                break;
            }
            else if(str_len > 0)
            {
                printf("choose = %d\r\n", innovate_recv.choose);
            }
            innovate_use.choose = innovate_recv.choose;
            // pthread_mutex_unlock(&tcp_recv_control_mutex);

            // system_delay_ms(10);
        }
    }
}