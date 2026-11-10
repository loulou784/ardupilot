#include <AP_HAL/AP_HAL.h>

#include <errno.h>
#include <sys/neutrino.h>
#include <time.h>

#include "Semaphores.h"
#include "Scheduler.h"

extern const AP_HAL::HAL& hal;

using namespace QNX;

/*
  QNX optimization: semaphore waits with monotonic, high-resolution
  timeouts.

  - Semaphore::take() blocks in pthread_mutex_timedlock_monotonic(),
    where AP_HAL_Linux polls every 200 us: the waiter wakes as soon as the
    semaphore is given, rather than at its next poll.
  - BinarySemaphore times out on CLOCK_MONOTONIC, where AP_HAL_Linux uses
    CLOCK_REALTIME, which jumps when the system clock is set, eg. from
    GPS time through Util::set_hw_rtc().
  - With the high_resolution_timer ability (see Scheduler.cpp), each
    timeout gets a tolerance below the clock tick, so that it ends on
    time rather than on the next 1 ms tick.
 */
namespace {

struct timespec deadline_after_us(uint64_t usec)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    const uint64_t nsec = uint64_t(ts.tv_nsec) + (usec % 1000000ULL) * 1000ULL;
    ts.tv_sec += usec / 1000000ULL + nsec / 1000000000ULL;
    ts.tv_nsec = nsec % 1000000000ULL;
    return ts;
}

#if AP_HAL_QNX_HIGH_RES_TIMER_ENABLED
bool high_res_timeouts_permitted = true;
#endif

// make the thread's next kernel timeout a high-resolution one, if the
// process may
void high_res_next_timeout()
{
#if AP_HAL_QNX_HIGH_RES_TIMER_ENABLED
    if (__atomic_load_n(&high_res_timeouts_permitted, __ATOMIC_RELAXED)) {
        uint64_t tolerance_ns = 1000;
        if (TimerTimeout(CLOCK_MONOTONIC, TIMER_TOLERANCE, nullptr, &tolerance_ns, nullptr) == -1) {
            __atomic_store_n(&high_res_timeouts_permitted, false, __ATOMIC_RELAXED);
        }
    }
#endif
}

}  // namespace

// construct a semaphore
Semaphore::Semaphore()
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&_lock, &attr);
}

bool Semaphore::give()
{
    return pthread_mutex_unlock(&_lock) == 0;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        return pthread_mutex_lock(&_lock) == 0;
    }
    if (take_nonblocking()) {
        return true;
    }
    if (timeout_ms == 0) {
        return false;
    }
    const struct timespec deadline = deadline_after_us(uint64_t(timeout_ms) * 1000ULL);
    high_res_next_timeout();
    return pthread_mutex_timedlock_monotonic(&_lock, &deadline) == 0;
}

bool Semaphore::take_nonblocking()
{
    return pthread_mutex_trylock(&_lock) == 0;
}

/*
  binary semaphore using pthread condition variables
 */

BinarySemaphore::BinarySemaphore(bool initial_state) :
    AP_HAL::BinarySemaphore(initial_state)
{
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    pthread_cond_init(&cond, &attr);
    pthread_condattr_destroy(&attr);
    pending = initial_state;
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
    WITH_SEMAPHORE(mtx);
    if (!pending) {
        const struct timespec deadline = deadline_after_us(timeout_us);
        // wait again after a spurious wakeup, which AP_HAL_Linux reports
        // as a signal
        while (!pending) {
            high_res_next_timeout();
            const int ret = pthread_cond_timedwait(&cond, &mtx._lock, &deadline);
            if (ret != 0 && ret != EINTR) {
                break;
            }
        }
        if (!pending) {
            return false;
        }
    }
    pending = false;
    return true;
}

bool BinarySemaphore::wait_blocking(void)
{
    WITH_SEMAPHORE(mtx);
    while (!pending) {
        if (pthread_cond_wait(&cond, &mtx._lock) != 0) {
            return false;
        }
    }
    pending = false;
    return true;
}

void BinarySemaphore::signal(void)
{
    WITH_SEMAPHORE(mtx);
    if (!pending) {
        pending = true;
        pthread_cond_signal(&cond);
    }
}
