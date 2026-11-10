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
*/
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

class SchedulerTiming {
public:
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
    hal.console->printf("SchedulerTiming: done\n");
    done = true;
}

AP_HAL_MAIN();
