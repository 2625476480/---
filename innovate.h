#ifndef _INNOVATE_H_
#define _INNOVATE_H
#include "zf_common_headfile.h"

typedef struct __attribute__((packed)) 
{
    int8_t choose;
} innovate_typedef;

void innovate_recv_thd_entry(void);
void close_innovate_recv(void);

#endif