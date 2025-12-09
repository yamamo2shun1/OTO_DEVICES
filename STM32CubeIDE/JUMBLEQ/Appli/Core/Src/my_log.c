/*
 * my_log.c
 *
 *  Created on: 2025/11/27
 *      Author: Shunichi Yamamoto
 */

#include "my_log.h"
#include "SEGGER_RTT.h"

int my_printf(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int ret = SEGGER_RTT_vprintf(0, fmt, &args);  // または args
    va_end(args);
    return ret;
}
