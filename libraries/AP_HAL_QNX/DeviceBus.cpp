#include "DeviceBus.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <AP_HAL/AP_HAL.h>

using namespace QNX;

extern const AP_HAL::HAL& hal;

namespace {

uint64_t now_monotonic_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}

class CallbacksLock {
public:
    CallbacksLock(pthread_mutex_t &m) : _m(m) { pthread_mutex_lock(&_m); }
    ~CallbacksLock() { pthread_mutex_unlock(&_m); }
private:
    pthread_mutex_t &_m;
};

}

DeviceBus::DeviceBus(AP_HAL::Device::BusType bus_type, uint8_t bus)
    : _bus_type(bus_type)
    , _bus(bus)
{
}

DeviceBus::~DeviceBus()
{
    teardown();
    for (Callback *c = _callbacks; c != nullptr;) {
        Callback *next = c->next;
        delete c;
        c = next;
    }
}

AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    Callback *callback = NEW_NOTHROW Callback;
    if (callback == nullptr) {
        // as on Linux: a sensor that silently never runs is worse
        AP_HAL::panic("Could not create periodic callback");
    }
    callback->cb = cb;
    callback->period_ns = uint64_t(period_usec) * 1000ULL;
    callback->next_ns = now_monotonic_ns() + callback->period_ns;
    callback->removed = false;
    callback->next = nullptr;

    {
        CallbacksLock lock(_callbacks_mutex);
        // run in the order registered, when several are due together
        Callback **tail = &_callbacks;
        while (*tail != nullptr) {
            tail = &(*tail)->next;
        }
        *tail = callback;
        _start_thread();
    }
    _timer.wakeup();

    return static_cast<AP_HAL::Device::PeriodicHandle>(callback);
}

DeviceBus::Callback *DeviceBus::_find(AP_HAL::Device::PeriodicHandle h)
{
    for (Callback *c = _callbacks; c != nullptr; c = c->next) {
        if (c == h && !c->removed) {
            return c;
        }
    }
    return nullptr;
}

bool DeviceBus::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h,
                                         uint32_t period_usec)
{
    {
        CallbacksLock lock(_callbacks_mutex);
        Callback *c = _find(h);
        if (c == nullptr) {
            return false;
        }
        c->period_ns = uint64_t(period_usec) * 1000ULL;
        c->next_ns = now_monotonic_ns() + c->period_ns;
    }
    if (!_thread.is_current_thread()) {
        _timer.wakeup();
    }
    return true;
}

bool DeviceBus::unregister_callback(AP_HAL::Device::PeriodicHandle h)
{
    CallbacksLock lock(_callbacks_mutex);
    Callback *c = _find(h);
    if (c == nullptr) {
        return false;
    }
    // freed by the bus thread, which may be running it right now
    c->removed = true;
    return true;
}

void DeviceBus::_start_thread()
{
    if (_thread.is_started()) {
        return;
    }
    if (!_timer.open()) {
        AP_HAL::panic("DeviceBus: failed to create the timer for bus %u: %s",
                      unsigned(_bus), strerror(errno));
    }
    char name[16];
    snprintf(name, sizeof(name), "ap-%s-%u",
             _bus_type == AP_HAL::Device::BUS_TYPE_SPI ? "spi" : "i2c",
             unsigned(_bus));
    _thread.set_stack_size(AP_QNX_SENSORS_STACK_SIZE);
    if (!_thread.start(name, AP_QNX_SENSORS_SCHED_POLICY, AP_QNX_SENSORS_SCHED_PRIO)) {
        AP_HAL::panic("DeviceBus: failed to start thread %s", name);
    }
}

DeviceBus::Callback *DeviceBus::_next_due(uint64_t now_ns, Callback *after)
{
    CallbacksLock lock(_callbacks_mutex);
    for (Callback *c = after != nullptr ? after->next : _callbacks; c != nullptr; c = c->next) {
        if (c->removed || c->next_ns > now_ns) {
            continue;
        }
        // skip the runs it missed, keeping its phase
        const uint64_t late_ns = now_ns - c->next_ns;
        if (c->period_ns == 0) {
            c->next_ns = now_ns;
        } else {
            c->next_ns += (late_ns / c->period_ns + 1) * c->period_ns;
        }
        return c;
    }
    return nullptr;
}

bool DeviceBus::_earliest(uint64_t &next_ns)
{
    CallbacksLock lock(_callbacks_mutex);
    bool found = false;
    for (Callback **p = &_callbacks; *p != nullptr;) {
        Callback *c = *p;
        if (c->removed) {
            *p = c->next;
            delete c;
            continue;
        }
        if (!found || c->next_ns < next_ns) {
            next_ns = c->next_ns;
            found = true;
        }
        p = &c->next;
    }
    return found;
}

void DeviceBus::_thread_main()
{
    while (!_should_exit) {
        // run every callback that is due, each with the bus semaphore
        // held, and each once per pass, so that a slow callback cannot
        // keep the ones after it from running
        const uint64_t now_ns = now_monotonic_ns();
        for (Callback *c = _next_due(now_ns, nullptr); c != nullptr && !_should_exit;
             c = _next_due(now_ns, c)) {
            // only this thread frees callbacks, so c stays valid; copy the
            // functor anyway, as other threads may change the list
            AP_HAL::Device::PeriodicCb cb;
            {
                CallbacksLock lock(_callbacks_mutex);
                cb = c->cb;
            }
            {
                WITH_SEMAPHORE(_semaphore);
                cb();
            }
        }

        uint64_t next_ns;
        if (_earliest(next_ns)) {
            if (next_ns <= now_monotonic_ns()) {
                continue;
            }
            if (!_timer.arm_at_ns(next_ns)) {
                AP_HAL::panic("DeviceBus: failed to arm the timer of bus %u: %s",
                              unsigned(_bus), strerror(errno));
            }
        } else {
            _timer.disarm();
        }

        // until the timer fires, or another thread changes the callbacks;
        // a stale timer pulse only makes the loop run once for nothing
        if (_timer.wait() == PulseTimer::Pulse::ERROR) {
            AP_HAL::panic("DeviceBus: failed to wait on bus %u: %s",
                          unsigned(_bus), strerror(errno));
        }
    }
}

void DeviceBus::teardown()
{
    if (!_thread.is_started()) {
        return;
    }
    _should_exit = true;
    _timer.wakeup();
    _thread.join();
}
