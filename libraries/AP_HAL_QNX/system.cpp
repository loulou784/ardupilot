#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QNX

#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/system.h>
#include <AP_Math/div1000.h>

extern const AP_HAL::HAL& hal;

namespace AP_HAL {

static struct {
    uint64_t start_time_ns;
} state;

static uint64_t ts_to_nsec(struct timespec &ts)
{
    return ts.tv_sec*1000000000ULL + ts.tv_nsec;
}

void init()
{
    struct timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    state.start_time_ns = ts_to_nsec(ts);
}

void WEAK panic(const char *errormsg, ...)
{
    va_list ap;

    va_start(ap, errormsg);
    vdprintf(1, errormsg, ap);
    va_end(ap);
    UNUSED_RESULT(write(1, "\n", 1));

    if (hal.scheduler != nullptr) {
        hal.scheduler->delay_microseconds(10000);
    }
    exit(1);
}

uint32_t micros()
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis()
{
    return millis64() & 0xFFFFFFFF;
}

uint64_t micros64()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_div1000(ts_to_nsec(ts) - state.start_time_ns);
}

uint64_t millis64()
{
    return uint64_div1000(micros64());
}

} // namespace AP_HAL

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QNX
