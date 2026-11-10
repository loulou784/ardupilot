/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  SchedulerTiming: measure how precisely the HAL scheduler keeps time,
  using only the AP_HAL interfaces, so that the same test runs on every
  HAL (eg. to compare the QNX and Linux HALs on the same board):

  - the period of the timer thread, through a registered timer process
  - how long delay_microseconds() really waits, from the main thread
  - how soon a thread waiting on a semaphore wakes when another thread
    gives or signals it, and how long semaphore timeouts really last
*/
#include <string.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static constexpr uint32_t TIMER_SAMPLES = 5000;
static uint32_t timer_period_us[TIMER_SAMPLES];
static volatile uint32_t timer_count;
static uint64_t timer_last_us;

struct Stats {
    uint32_t n;
    uint64_t sum;
    uint32_t min;
    uint32_t max;
    void add(uint32_t v) {
        if (n == 0 || v < min) {
            min = v;
        }
        if (v > max) {
            max = v;
        }
        sum += v;
        n++;
    }
    float mean() const { return n ? float(sum) / n : 0; }
};

// what the helper thread is asked to do with the semaphores
enum class Request : uint8_t {
    NONE,
    HOLD_THEN_GIVE,     // take sem, give it 300 us later
    HOLD_UNTIL_TOLD,    // take sem, give it when request becomes NONE
    SIGNAL_LATER,       // signal bsem 300 us later
};
static volatile Request request;
static volatile bool helper_holding;
static volatile uint64_t release_us;
static HAL_Semaphore sem;
static HAL_BinarySemaphore bsem;

static void busy_wait_us(uint32_t us)
{
    const uint64_t until = AP_HAL::micros64() + us;
    while (AP_HAL::micros64() < until) {
    }
}

class SchedulerTiming {
public:
    void helper_thread() {
        while (true) {
            switch (request) {
            case Request::NONE:
                hal.scheduler->delay_microseconds(100);
                break;
            case Request::HOLD_THEN_GIVE:
                sem.take_blocking();
                helper_holding = true;
                busy_wait_us(300);
                release_us = AP_HAL::micros64();
                helper_holding = false;
                sem.give();
                request = Request::NONE;
                break;
            case Request::HOLD_UNTIL_TOLD:
                sem.take_blocking();
                helper_holding = true;
                while (request == Request::HOLD_UNTIL_TOLD) {
                    hal.scheduler->delay_microseconds(100);
                }
                helper_holding = false;
                sem.give();
                break;
            case Request::SIGNAL_LATER:
                busy_wait_us(300);
                release_us = AP_HAL::micros64();
                bsem.signal();
                request = Request::NONE;
                break;
            }
        }
    }

    void timer_tick() {
        const uint64_t now = AP_HAL::micros64();
        if (timer_last_us != 0 && timer_count < TIMER_SAMPLES) {
            timer_period_us[timer_count] = uint32_t(now - timer_last_us);
            timer_count = timer_count + 1;
        }
        timer_last_us = now;
    }
};

static SchedulerTiming timing;

// percentile of the samples, in us, from a 1 us histogram up to 4 ms
static uint32_t percentile(const uint32_t *samples, uint32_t n, float pct)
{
    static constexpr uint32_t BINS = 4000;
    static uint16_t histogram[BINS];
    memset(histogram, 0, sizeof(histogram));
    for (uint32_t i = 0; i < n; i++) {
        histogram[samples[i] < BINS ? samples[i] : BINS - 1]++;
    }
    const uint32_t wanted = uint32_t(n * pct / 100.0f);
    uint32_t seen = 0;
    for (uint32_t bin = 0; bin < BINS; bin++) {
        seen += histogram[bin];
        if (seen > wanted) {
            return bin;
        }
    }
    return BINS - 1;
}

static void report_timer_thread()
{
    Stats s {};
    uint32_t over_1500 = 0;
    for (uint32_t i = 0; i < timer_count; i++) {
        s.add(timer_period_us[i]);
        if (timer_period_us[i] > 1500) {
            over_1500++;
        }
    }
    hal.console->printf("timer thread period (%u samples): mean %.1f min %u max %u us, %.1f Hz; %u over 1500 us\n",
                        unsigned(s.n), s.mean(), unsigned(s.min), unsigned(s.max),
                        s.mean() > 0 ? 1.0e6f / s.mean() : 0.0f, unsigned(over_1500));
    hal.console->printf("timer thread period percentiles: 0.1%% %u, 1%% %u, 50%% %u, 99%% %u, 99.9%% %u us\n",
                        unsigned(percentile(timer_period_us, timer_count, 0.1f)),
                        unsigned(percentile(timer_period_us, timer_count, 1.0f)),
                        unsigned(percentile(timer_period_us, timer_count, 50.0f)),
                        unsigned(percentile(timer_period_us, timer_count, 99.0f)),
                        unsigned(percentile(timer_period_us, timer_count, 99.9f)));
}

static void report(const char *what, const Stats &s)
{
    hal.console->printf("%s: mean %7.1f min %5u max %5u us\n",
                        what, s.mean(), unsigned(s.min), unsigned(s.max));
}

static void wait_for_helper_idle()
{
    while (request != Request::NONE || helper_holding) {
        hal.scheduler->delay_microseconds(100);
    }
}

static void report_semaphores()
{
    static constexpr uint16_t SAMPLES = 200;
    Stats handoff {}, take_timeout {}, signal_wake {}, wait_timeout {};
    for (uint16_t i = 0; i < SAMPLES; i++) {
        // how soon take() returns once the holder gives the semaphore
        request = Request::HOLD_THEN_GIVE;
        while (!helper_holding) {
        }
        if (sem.take(100)) {
            handoff.add(uint32_t(AP_HAL::micros64() - release_us));
            sem.give();
        }
        wait_for_helper_idle();

        // how long take(1) waits for a semaphore that is never given
        request = Request::HOLD_UNTIL_TOLD;
        while (!helper_holding) {
        }
        uint64_t t0 = AP_HAL::micros64();
        if (!sem.take(1)) {
            take_timeout.add(uint32_t(AP_HAL::micros64() - t0));
        }
        request = Request::NONE;
        wait_for_helper_idle();

        // how soon wait() returns once the binary semaphore is signalled
        request = Request::SIGNAL_LATER;
        if (bsem.wait(100000)) {
            signal_wake.add(uint32_t(AP_HAL::micros64() - release_us));
        }
        wait_for_helper_idle();

        // how long wait(500) waits for a signal that never comes
        t0 = AP_HAL::micros64();
        if (!bsem.wait(500)) {
            wait_timeout.add(uint32_t(AP_HAL::micros64() - t0));
        }
    }
    report("Semaphore::take() wake after give", handoff);
    report("Semaphore::take(1) timeout       ", take_timeout);
    report("BinarySemaphore::wait() wake     ", signal_wake);
    report("BinarySemaphore::wait(500) timeout", wait_timeout);
}

static void report_delays()
{
    static const uint16_t requests_us[] = { 50, 100, 250, 500, 1000, 2500 };
    for (const uint16_t req : requests_us) {
        Stats s {};
        for (uint16_t i = 0; i < 500; i++) {
            const uint64_t t0 = AP_HAL::micros64();
            hal.scheduler->delay_microseconds(req);
            s.add(uint32_t(AP_HAL::micros64() - t0));
        }
        hal.console->printf("delay_microseconds(%4u): mean %7.1f min %5u max %5u us\n",
                            unsigned(req), s.mean(), unsigned(s.min), unsigned(s.max));
    }
}

void setup()
{
    hal.console->printf("SchedulerTiming: measuring the timer thread for %u samples\n",
                        unsigned(TIMER_SAMPLES));
    hal.scheduler->register_timer_process(FUNCTOR_BIND(&timing, &SchedulerTiming::timer_tick, void));
    if (!hal.scheduler->thread_create(FUNCTOR_BIND(&timing, &SchedulerTiming::helper_thread, void),
                                      "timing", 8192, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
        hal.console->printf("SchedulerTiming: failed to create the helper thread\n");
    }
}

void loop()
{
    static bool done;
    if (done) {
        hal.scheduler->delay(1000);
        return;
    }
    // let the timer thread collect its samples, then time the delays
    while (timer_count < TIMER_SAMPLES) {
        hal.scheduler->delay(100);
    }
    report_timer_thread();
    report_delays();
    report_semaphores();
    hal.console->printf("SchedulerTiming: done\n");
    done = true;
}

AP_HAL_MAIN();
