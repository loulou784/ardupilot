
#include "UARTDriver.h"
#include <AP_Common/ExpandingString.h>

QNX::UARTDriver::UARTDriver() {}

/* QNX implementations of virtual methods */
void QNX::UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void QNX::UARTDriver::_end() {}
void QNX::UARTDriver::_flush() {}
bool QNX::UARTDriver::is_initialized() { return false; }
bool QNX::UARTDriver::tx_pending() { return false; }

uint32_t QNX::UARTDriver::_available() { return 0; }
uint32_t QNX::UARTDriver::txspace() { return 1; }
bool QNX::UARTDriver::_discard_input() { return false; }
size_t QNX::UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    return size;
}
ssize_t QNX::UARTDriver::_read(uint8_t *buffer, uint16_t size)
{
    return 0;
}

#if HAL_UART_STATS_ENABLED
void QNX::UARTDriver::uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms)
{
    str.printf("QNX\n");
}
#endif
