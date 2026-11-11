#pragma once

#include <pthread.h>
#include <stdint.h>

#include <AP_HAL/Device.h>

#include "PulseTimer.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include "Thread.h"

namespace QNX {

/*
  One thread per I2C or SPI bus, which runs the periodic callbacks of the
  devices on that bus with the bus semaphore held, as AP_HAL_Linux's
  PollerThread does.

  QNX difference: Linux gives each callback a timerfd and waits for them
  with epoll(), which QNX does not have.  Here the thread keeps the next
  run time of each callback, as AP_HAL_ChibiOS's DeviceBus does, and
  sleeps until the earliest with one high-resolution timer (PulseTimer),
  which other threads can wake when they add or change a callback.  A
  callback that falls behind skips the runs it missed rather than running
  them back to back, and keeps its phase.
 */
class DeviceBus {
public:
    DeviceBus(AP_HAL::Device::BusType bus_type, uint8_t bus);
    ~DeviceBus();

    // held around every transfer and every callback on the bus
    Semaphore &semaphore() { return _semaphore; }

    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb cb);
    bool adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                  uint32_t period_usec);
    bool unregister_callback(AP_HAL::Device::PeriodicHandle h);

    // stop the thread, and wait until it has stopped; callbacks no longer
    // run after this
    void teardown();

private:
    struct Callback {
        AP_HAL::Device::PeriodicCb cb;
        uint64_t period_ns;
        uint64_t next_ns;
        bool removed;
        Callback *next;
    };

    void _start_thread();
    void _thread_main();
    Callback *_find(AP_HAL::Device::PeriodicHandle h);

    // take the next callback after "after" (from the start if null) that
    // is due at now_ns, and schedule its next run
    Callback *_next_due(uint64_t now_ns, Callback *after);
    // free removed callbacks; false if no callback is left
    bool _earliest(uint64_t &next_ns);

    const AP_HAL::Device::BusType _bus_type;
    const uint8_t _bus;
    Semaphore _semaphore;

    // protects the callback list, which other threads change
    pthread_mutex_t _callbacks_mutex = PTHREAD_MUTEX_INITIALIZER;
    Callback *_callbacks = nullptr;

    PulseTimer _timer;
    Thread _thread{FUNCTOR_BIND_MEMBER(&DeviceBus::_thread_main, void)};
    volatile bool _should_exit = false;
};

}
