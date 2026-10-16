#include "control_console.h"

static UART_HandleTypeDef *console_uart;

void ControlConsole_Init(UART_HandleTypeDef *uart)
{
    console_uart = uart;
}

ControlCommand ControlConsole_Poll(void)
{
    uint8_t byte;

    if (console_uart == NULL ||
        HAL_UART_Receive(console_uart, &byte, 1U, 0U) != HAL_OK)
    {
        return CONTROL_COMMAND_NONE;
    }

    switch (byte)
    {
        case 'a':
        case 'A':
            return CONTROL_COMMAND_ARM;
        case 'd':
        case 'D':
            return CONTROL_COMMAND_DISARM;
        case 's':
        case 'S':
            return CONTROL_COMMAND_STATUS;
        default:
            return CONTROL_COMMAND_NONE;
    }
}
