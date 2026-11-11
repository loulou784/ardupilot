#pragma once

#include <stdint.h>
#include <time.h>

#include "AP_HAL_QNX.h"

// QNX optimization: sleep with high-resolution timers rather than on the
// clock tick; set to 0 to sleep as on Linux, eg. for a baseline benchmark
#ifndef AP_HAL_QNX_HIGH_RES_TIMER_ENABLED
#define AP_HAL_QNX_HIGH_RES_TIMER_ENABLED 1
#endif

namespace QNX {

/*
  A QNX timer that sends a pulse to a channel of its own, which one thread
  waits on; other threads can wake that thread with a pulse too.

  QNX optimization: high-resolution timers.  Kernel timeouts and
  nanosleep() on QNX end on a clock tick, 1 ms by default, and QNX 8
  cannot change the tick at run time (ClockPeriod() reports ENOTSUP).  A
  QNX 8 timer with a tolerance below the tick is a high-resolution timer:
  the kernel programs it on its own, and it fires within a few
  microseconds of the requested time.

  Setting a tolerance below the tick needs the high_resolution_timer
  ability, which root has.  Give it to another user with, eg.
      on -A nonroot,allow,high_resolution_timer -u <user> arducopter ...
  Without it, the timer still works, on the clock tick, and the first
  timer to find that out prints a warning.
 */
class PulseTimer {
public:
    // what a pulse was for
    enum class Pulse : uint8_t {
        TIMER,      // the timer expired
        WAKEUP,     // another thread called wakeup()
        ERROR,      // waiting failed; see errno
    };

    ~PulseTimer() { close(); }

    // create the channel, the connection and the timer; false if any of
    // them could not be created
    bool open();
    void close();

    bool is_open() const { return _have_timer; }

    // true if the timer fires within a few microseconds of the requested
    // time, false if it ends on the clock tick
    bool high_resolution() const { return _high_resolution; }

    // fire once, after ns nanoseconds or at the CLOCK_MONOTONIC time ns
    bool arm_after_ns(uint64_t ns) { return _arm(ns, 0); }
    bool arm_at_ns(uint64_t ns) { return _arm(ns, TIMER_ABSTIME); }

    // cancel the timer; a pulse already sent stays queued
    bool disarm();

    // from another thread: end the current or next wait() with WAKEUP
    bool wakeup();

    // wait for the next pulse, the timer's or a wakeup
    Pulse wait();

private:
    bool _arm(uint64_t ns, int flags);

    int _chid = -1;
    int _coid = -1;
    bool _have_timer = false;
    bool _high_resolution = false;
    timer_t _timer {};
};

}
