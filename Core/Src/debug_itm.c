#include "stm32wbxx_hal.h"
#include <stdint.h>

int __io_putchar(int ch)
{
    ITM_SendChar((uint32_t)ch);
    return ch;
}
