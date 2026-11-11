#include "PulseTimer.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/neutrino.h>
#include <sys/siginfo.h>

using namespace QNX;

namespace {

// how late a high-resolution timer may fire
constexpr long HIGH_RES_TIMER_TOLERANCE_NS = 1000;

constexpr int PULSE_CODE_TIMER = _PULSE_CODE_MINAVAIL;
constexpr int PULSE_CODE_WAKEUP = _PULSE_CODE_MINAVAIL + 1;

bool warned_no_high_res_timer;

}

bool PulseTimer::open()
{
    if (_have_timer) {
        return true;
    }
    // a thread that receives a pulse takes the pulse's priority, unless
    // its channel has a fixed priority: without it, the first sleep
    // dropped every scheduler thread to the timer pulse's priority, 10
    _chid = ChannelCreate(_NTO_CHF_PRIVATE | _NTO_CHF_FIXED_PRIORITY);
    if (_chid == -1) {
        close();
        return false;
    }
    _coid = ConnectAttach(0, 0, _chid, _NTO_SIDE_CHANNEL, 0);
    if (_coid == -1) {
        close();
        return false;
    }
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, _coid, SIGEV_PULSE_PRIO_INHERIT, PULSE_CODE_TIMER, 0);
    if (timer_create(CLOCK_MONOTONIC, &ev, &_timer) == -1) {
        close();
        return false;
    }
    _have_timer = true;

#if AP_HAL_QNX_HIGH_RES_TIMER_ENABLED
    struct itimerspec tolerance {};
    tolerance.it_value.tv_nsec = HIGH_RES_TIMER_TOLERANCE_NS;
    if (timer_settime(_timer, TIMER_TOLERANCE, &tolerance, nullptr) == 0) {
        _high_resolution = true;
    } else if (!__atomic_exchange_n(&warned_no_high_res_timer, true, __ATOMIC_RELAXED)) {
        ::fprintf(stderr, "QNX: no high-resolution timers (%s), so sleeps and device "
                  "callbacks end on the clock tick; they need the high_resolution_timer "
                  "ability\n", strerror(errno));
    }
#endif
    return true;
}

void PulseTimer::close()
{
    if (_have_timer) {
        timer_delete(_timer);
        _have_timer = false;
    }
    _high_resolution = false;
    if (_coid != -1) {
        ConnectDetach(_coid);
        _coid = -1;
    }
    if (_chid != -1) {
        ChannelDestroy(_chid);
        _chid = -1;
    }
}

bool PulseTimer::_arm(uint64_t ns, int flags)
{
    if (!_have_timer) {
        return false;
    }
    struct itimerspec expiry {};
    expiry.it_value.tv_sec = ns / 1000000000ULL;
    expiry.it_value.tv_nsec = ns % 1000000000ULL;
    if (expiry.it_value.tv_sec == 0 && expiry.it_value.tv_nsec == 0) {
        // a zero expiry disarms the timer: fire as soon as possible instead
        expiry.it_value.tv_nsec = 1;
    }
    return timer_settime(_timer, flags, &expiry, nullptr) == 0;
}

bool PulseTimer::disarm()
{
    if (!_have_timer) {
        return false;
    }
    struct itimerspec expiry {};
    return timer_settime(_timer, 0, &expiry, nullptr) == 0;
}

bool PulseTimer::wakeup()
{
    if (_coid == -1) {
        return false;
    }
    return MsgSendPulse(_coid, -1, PULSE_CODE_WAKEUP, 0) == 0;
}

PulseTimer::Pulse PulseTimer::wait()
{
    if (_chid == -1) {
        errno = EBADF;
        return Pulse::ERROR;
    }
    struct _pulse pulse;
    while (MsgReceivePulse(_chid, &pulse, sizeof(pulse), nullptr) == -1) {
        if (errno != EINTR) {
            return Pulse::ERROR;
        }
    }
    return pulse.code == PULSE_CODE_WAKEUP ? Pulse::WAKEUP : Pulse::TIMER;
}
