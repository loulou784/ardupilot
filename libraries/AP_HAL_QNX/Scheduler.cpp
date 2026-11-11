#include "Scheduler.h"

#include <algorithm>
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/siginfo.h>
#include <sys/syspage.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include "RCInput.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "Util.h"

using namespace QNX;

extern const AP_HAL::HAL& hal;

#define APM_QNX_MAX_PRIORITY          20
#define APM_QNX_TIMER_PRIORITY        15
#define APM_QNX_UART_PRIORITY         14
#define APM_QNX_NET_PRIORITY          14
#define APM_QNX_RCIN_PRIORITY         13
#define APM_QNX_MAIN_PRIORITY         12
#define APM_QNX_IO_PRIORITY           10
#define APM_QNX_SCRIPTING_PRIORITY     1

#define APM_QNX_TIMER_RATE            1000
#define APM_QNX_UART_RATE             100

#ifndef APM_QNX_RCIN_RATE
#define APM_QNX_RCIN_RATE             100
#endif  // APM_QNX_RCIN_RATE

#ifndef APM_QNX_IO_RATE
#define APM_QNX_IO_RATE               50
#endif  // APM_QNX_IO_RATE

#define SCHED_THREAD(name_, UPPER_NAME_)                        \
    {                                                           \
        .name = "ap-" #name_,                                   \
        .thread = &_##name_##_thread,                           \
        .policy = SCHED_FIFO,                                   \
        .prio = APM_QNX_##UPPER_NAME_##_PRIORITY,             \
        .rate = APM_QNX_##UPPER_NAME_##_RATE,                 \
    }

Scheduler::Scheduler()
{
    _cpu_affinity = 0;
}


void Scheduler::init_realtime()
{
#if APM_BUILD_TYPE(APM_BUILD_Replay)
    // we don't run Replay in real-time...
    return;
#endif
    // Unlike Linux, QNX gives SCHED_FIFO to users other than root, up to
    // priority 63 by default, so examples and tools run in realtime too,
    // and so does ArduPilot run as an ordinary user.

    // locking memory needs root on QNX, as on Linux; without it ArduPilot
    // still runs, as on Linux
    mlockall(MCL_CURRENT|MCL_FUTURE);

    struct sched_param param = { .sched_priority = APM_QNX_MAIN_PRIORITY };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == -1) {
        AP_HAL::panic("Scheduler: failed to set scheduling parameters: %s",
                      strerror(errno));
    }
}

void Scheduler::init_cpu_affinity()
{
    if (_cpu_affinity == 0) {
        return;
    }

    /*
      QNX has no sched_setaffinity(): set the runmask of the main thread,
      and make it the inherit mask, so that every thread created from here
      on runs on the same CPUs, as on Linux.  The kernel needs both masks
      sized for exactly the number of CPUs in the system.
     */
    const int num_cpu = _syspage_ptr->num_cpu;
    const int size = RMSK_SIZE(num_cpu);
    // room for both masks on systems of up to 256 CPUs
    unsigned rmsk[1 + 2 * RMSK_SIZE(256)] {};
    if (size > (int)RMSK_SIZE(256)) {
        AP_HAL::panic("Failed to set affinity for main process: %d CPUs", num_cpu);
    }
    rmsk[0] = size;
    unsigned *runmask = &rmsk[1];
    unsigned *inherit_mask = &rmsk[1 + size];
    for (int cpu = 0; cpu < num_cpu && cpu < 32; cpu++) {
        if (_cpu_affinity & (1U << cpu)) {
            RMSK_SET(cpu, runmask);
            RMSK_SET(cpu, inherit_mask);
        }
    }

    if (ThreadCtl(_NTO_TCTL_RUNMASK_GET_AND_SET_INHERIT, rmsk) == -1) {
        /*
          Unlike Linux, QNX 8 runs threads on scheduling clusters, and
          only accepts a runmask that matches one.  Unless the boot image
          defines more, the clusters are each single CPU and all CPUs, so
          eg. "-c 2-3" fails with EINVAL.
         */
        if (errno == EINVAL) {
            AP_HAL::panic("Failed to set affinity for main process: CPU set 0x%x "
                          "is not a QNX scheduling cluster; use one CPU or all CPUs, "
                          "or define the cluster in the boot image",
                          (unsigned)_cpu_affinity);
        }
        // %m is a glibc extension, so spell it out
        AP_HAL::panic("Failed to set affinity for main process: %s",
                      strerror(errno));
    }
}

void Scheduler::init()
{
    int ret;
    const struct sched_table {
        const char *name;
        SchedulerThread *thread;
        int policy;
        int prio;
        uint32_t rate;
    } sched_table[] = {
        SCHED_THREAD(timer, TIMER),
        SCHED_THREAD(uart, UART),
        SCHED_THREAD(rcin, RCIN),
        SCHED_THREAD(io, IO),
    };

    _main_ctx = pthread_self();

    init_realtime();
    init_cpu_affinity();

    /* set barrier to N + 1 threads: worker threads + main */
    unsigned n_threads = ARRAY_SIZE(sched_table) + 1;
    ret = pthread_barrier_init(&_initialized_barrier, nullptr, n_threads);
    if (ret) {
        AP_HAL::panic("Scheduler: Failed to initialise barrier object: %s",
                      strerror(ret));
    }

    for (size_t i = 0; i < ARRAY_SIZE(sched_table); i++) {
        const struct sched_table *t = &sched_table[i];

        t->thread->set_rate(t->rate);
        t->thread->set_stack_size(1024 * 1024);
        t->thread->start(t->name, t->policy, t->prio);
    }

#if defined(DEBUG_STACK) && DEBUG_STACK
    register_timer_process(FUNCTOR_BIND_MEMBER(&Scheduler::_debug_stack, void));
#endif
}

void Scheduler::_debug_stack()
{
    uint64_t now = AP_HAL::millis64();

    if (now - _last_stack_debug_msec > 5000) {
        fprintf(stderr, "Stack Usage:\n"
                "\ttimer = %zu\n"
                "\tio    = %zu\n"
                "\trcin  = %zu\n"
                "\tuart  = %zu\n",
                _timer_thread.get_stack_usage(),
                _io_thread.get_stack_usage(),
                _rcin_thread.get_stack_usage(),
                _uart_thread.get_stack_usage());
        _last_stack_debug_msec = now;
    }
}

#if AP_HAL_QNX_HIGH_RES_TIMER_ENABLED
/*
  QNX optimization: high-resolution sleeps.

  nanosleep() on QNX ends on a clock tick, 1 ms by default, and QNX 8
  cannot change the tick at run time (ClockPeriod() reports ENOTSUP), so
  a 250 us sleep takes 1 ms and a 2.5 ms one 3 ms.  A QNX 8 timer with a
  tolerance below the tick is a high-resolution timer: the kernel
  programs it on its own, and it fires within a few microseconds of the
  requested time.  Each thread that sleeps gets such a timer, which sends
  a pulse to a channel of its own that the thread waits on.

  Setting a tolerance below the tick needs the high_resolution_timer
  ability, which root has.  Give it to another user with, eg.
      on -A nonroot,allow,high_resolution_timer -u <user> arducopter ...
  Without it, sleeps fall back to nanosleep() and the clock tick.
 */
namespace {

// how late a high-resolution sleep may end
constexpr long HIGH_RES_TIMER_TOLERANCE_NS = 1000;

class SleepTimer {
public:
    ~SleepTimer() { close(); }

    // sleep for usec microseconds; false if this thread cannot use a
    // high-resolution timer, and must sleep some other way
    bool sleep(uint32_t usec);

private:
    bool open();
    void close();

    enum class State : uint8_t { UNTRIED, OPEN, FAILED };
    State _state = State::UNTRIED;
    int _chid = -1;
    int _coid = -1;
    bool _have_timer = false;
    timer_t _timer {};
};

thread_local SleepTimer sleep_timer;
bool warned_no_high_res_timer;

bool SleepTimer::open()
{
    // a thread that receives a pulse takes the pulse's priority, unless
    // its channel has a fixed priority: without it, the first sleep
    // dropped every scheduler thread to the timer pulse's priority, 10
    _chid = ChannelCreate(_NTO_CHF_PRIVATE | _NTO_CHF_FIXED_PRIORITY);
    if (_chid == -1) {
        return false;
    }
    _coid = ConnectAttach(0, 0, _chid, _NTO_SIDE_CHANNEL, 0);
    if (_coid == -1) {
        return false;
    }
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, _coid, SIGEV_PULSE_PRIO_INHERIT, _PULSE_CODE_MINAVAIL, 0);
    if (timer_create(CLOCK_MONOTONIC, &ev, &_timer) == -1) {
        return false;
    }
    _have_timer = true;

    struct itimerspec tolerance {};
    tolerance.it_value.tv_nsec = HIGH_RES_TIMER_TOLERANCE_NS;
    if (timer_settime(_timer, TIMER_TOLERANCE, &tolerance, nullptr) == -1) {
        if (!__atomic_exchange_n(&warned_no_high_res_timer, true, __ATOMIC_RELAXED)) {
            ::fprintf(stderr, "Scheduler: no high-resolution timers (%s), so sleeps end on the "
                      "clock tick; they need the high_resolution_timer ability\n",
                      strerror(errno));
        }
        return false;
    }
    return true;
}

void SleepTimer::close()
{
    if (_have_timer) {
        timer_delete(_timer);
        _have_timer = false;
    }
    if (_coid != -1) {
        ConnectDetach(_coid);
        _coid = -1;
    }
    if (_chid != -1) {
        ChannelDestroy(_chid);
        _chid = -1;
    }
}

bool SleepTimer::sleep(uint32_t usec)
{
    if (_state == State::UNTRIED) {
        _state = open() ? State::OPEN : State::FAILED;
        if (_state == State::FAILED) {
            close();
        }
    }
    if (_state != State::OPEN) {
        return false;
    }
    if (usec == 0) {
        return true;
    }

    struct itimerspec expiry {};
    expiry.it_value.tv_sec = usec / 1000000UL;
    expiry.it_value.tv_nsec = (usec % 1000000UL) * 1000UL;
    if (timer_settime(_timer, 0, &expiry, nullptr) == -1) {
        return false;
    }
    struct _pulse pulse;
    while (MsgReceivePulse(_chid, &pulse, sizeof(pulse), nullptr) == -1) {
        if (errno != EINTR) {
            // stop using this timer, so that a late pulse cannot end a
            // later sleep early
            _state = State::FAILED;
            close();
            return false;
        }
    }
    return true;
}

}  // namespace
#endif  // AP_HAL_QNX_HIGH_RES_TIMER_ENABLED

void Scheduler::microsleep(uint32_t usec)
{
#if AP_HAL_QNX_HIGH_RES_TIMER_ENABLED
    if (sleep_timer.sleep(usec)) {
        return;
    }
#endif
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) ;
}

void Scheduler::delay(uint16_t ms)
{
    if (_stopped_clock_usec) {
        return;
    }

    if (ms == 0) {
        return;
    }

    uint64_t now = AP_HAL::micros64();
    uint64_t end = now + 1000UL * ms + 1U;
    do {
        // this yields the CPU to other apps
        microsleep(MIN(1000UL, end-now));
        if (in_main_thread() && _min_delay_cb_ms <= ms) {
            call_delay_cb();
        }
        now = AP_HAL::micros64();
    } while (now < end);
}

void Scheduler::delay_microseconds(uint16_t us)
{
    if (_stopped_clock_usec) {
        return;
    }
    microsleep(us);
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }

    if (_num_timer_procs >= QNX_SCHEDULER_MAX_TIMER_PROCS) {
        hal.console->printf("Out of timer processes\n");
        return;
    }

    _timer_proc[_num_timer_procs] = proc;
    _num_timer_procs++;
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }

    if (_num_io_procs < QNX_SCHEDULER_MAX_IO_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        hal.console->printf("Out of IO processes\n");
    }
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::_timer_task()
{
    int i;

    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    // now call the timer based drivers
    for (i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void Scheduler::_run_io(void)
{
    _io_semaphore.take_blocking();

    // now call the IO based drivers
    for (int i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }

    _io_semaphore.give();
}

/*
  run timers for all UARTs
 */
void Scheduler::_run_uarts()
{
    // process any pending serial bytes
    for (uint8_t i=0;i<hal.num_serial; i++) {
        hal.serial(i)->_timer_tick();
    }
}

void Scheduler::_rcin_task()
{
    RCInput::from(hal.rcin)->_timer_tick();
}

void Scheduler::_uart_task()
{
    _run_uarts();
}

void Scheduler::_io_task()
{
    // process any pending storage writes
    hal.storage->_timer_tick();

    // run registered IO processes
    _run_io();
}

bool Scheduler::in_main_thread() const
{
    return pthread_equal(pthread_self(), _main_ctx);
}

void Scheduler::_wait_all_threads()
{
    int r = pthread_barrier_wait(&_initialized_barrier);
    if (r == PTHREAD_BARRIER_SERIAL_THREAD) {
        pthread_barrier_destroy(&_initialized_barrier);
    }
}

void Scheduler::set_system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::set_system_initialized called more than once");
    }

    _initialized = true;

    _wait_all_threads();
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    exit(1);
}

#if APM_BUILD_TYPE(APM_BUILD_Replay) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
void Scheduler::stop_clock(uint64_t time_usec)
{
    if (time_usec < _stopped_clock_usec) {
        ::fprintf(stderr, "Warning: setting time backwards from (%" PRIu64 ") to (%" PRIu64 ")\n", _stopped_clock_usec, time_usec);
        return;
    }

    _stopped_clock_usec = time_usec;
    _run_io();
}
#else
void Scheduler::stop_clock(uint64_t time_usec)
{
    // stop_clock() is not called outside of Replay, but we can't
    // guard it in the header because of the vehicle-dependent-library
    // checks in waf.
}
#endif

bool Scheduler::SchedulerThread::_run()
{
    _sched._wait_all_threads();

    return PeriodicThread::_run();
}

void Scheduler::teardown()
{
    _timer_thread.stop();
    _io_thread.stop();
    _rcin_thread.stop();
    _uart_thread.stop();

    _timer_thread.join();
    _io_thread.join();
    _rcin_thread.join();
    _uart_thread.join();
}

// calculates an integer to be used as the priority for a newly-created thread
uint8_t Scheduler::calculate_thread_priority(priority_base base, int8_t priority) const
{
    uint8_t thread_priority = APM_QNX_IO_PRIORITY;
    static const struct {
        priority_base base;
        uint8_t p;
    } priority_map[] = {
        { PRIORITY_BOOST, APM_QNX_MAIN_PRIORITY},
        { PRIORITY_MAIN, APM_QNX_MAIN_PRIORITY},
        { PRIORITY_SPI, AP_QNX_SENSORS_SCHED_PRIO},
        { PRIORITY_I2C, AP_QNX_SENSORS_SCHED_PRIO},
        { PRIORITY_CAN, APM_QNX_TIMER_PRIORITY},
        { PRIORITY_TIMER, APM_QNX_TIMER_PRIORITY},
        { PRIORITY_RCIN, APM_QNX_RCIN_PRIORITY},
        { PRIORITY_IO, APM_QNX_IO_PRIORITY},
        { PRIORITY_UART, APM_QNX_UART_PRIORITY},
        { PRIORITY_STORAGE, APM_QNX_IO_PRIORITY},
        { PRIORITY_SCRIPTING, APM_QNX_SCRIPTING_PRIORITY},
        { PRIORITY_NET, APM_QNX_NET_PRIORITY},
    };
    for (uint8_t i=0; i<ARRAY_SIZE(priority_map); i++) {
        if (priority_map[i].base == base) {
            thread_priority = constrain_int16(priority_map[i].p + priority, 1, APM_QNX_MAX_PRIORITY);
            break;
        }
    }

    return thread_priority;
}

/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t stack_size, priority_base base, int8_t priority)
{
    Thread *thread = NEW_NOTHROW Thread{(Thread::task_t)proc};
    if (!thread) {
        return false;
    }

    const uint8_t thread_priority = calculate_thread_priority(base, priority);

    // Add 256k to HAL-independent requested stack size
    thread->set_stack_size(256 * 1024 + stack_size);

    /*
     * We should probably store the thread handlers and join() when exiting,
     * but let's the thread manage itself for now.
     */
    thread->set_auto_free(true);

    if (!thread->start(name, SCHED_FIFO, thread_priority)) {
        delete thread;
        return false;
    }

    return true;
}
