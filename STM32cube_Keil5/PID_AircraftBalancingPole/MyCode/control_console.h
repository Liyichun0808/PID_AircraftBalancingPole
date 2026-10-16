#ifndef __CONTROL_CONSOLE_H
#define __CONTROL_CONSOLE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum
{
    CONTROL_COMMAND_NONE = 0,
    CONTROL_COMMAND_ARM,
    CONTROL_COMMAND_DISARM,
    CONTROL_COMMAND_STATUS
} ControlCommand;

void ControlConsole_Init(UART_HandleTypeDef *uart);
ControlCommand ControlConsole_Poll(void);

#endif
