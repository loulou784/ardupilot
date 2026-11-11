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
  GPIOTest: check a board's GPIO driver through the AP_HAL interfaces, on
  one pin, which this test drives: it must be free, with nothing
  connected that could drive it too.  For another pin than the default,
  configure with eg. --define GPIO_TEST_PIN=6.

  - the pin as an output: write, read back, toggle, and the time each
    takes, through hal.gpio and through a DigitalSource
  - pin interrupts, if the HAL has them: the pin's own edges, as it
    toggles, are counted and timed; then the interrupt is detached
  - the pin is left as an input
*/
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#ifndef GPIO_TEST_PIN
#define GPIO_TEST_PIN 26
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

constexpr uint8_t pin = GPIO_TEST_PIN;
constexpr uint16_t EDGES = 200;

volatile uint16_t interrupts;
volatile uint16_t wrong_state;
volatile uint32_t edge_us;
uint32_t latency_sum_us, latency_max_us;
uint16_t latency_count;

class EdgeCounter {
public:
    void on_edge(uint8_t p, bool state, uint32_t timestamp);
};
EdgeCounter edge_counter;

void EdgeCounter::on_edge(uint8_t p, bool state, uint32_t timestamp)
{
    if (p != pin) {
        return;
    }
    // the pin was written to state just before
    if (state != (interrupts % 2 == 0)) {
        wrong_state = wrong_state + 1;
    }
    interrupts = interrupts + 1;
    const uint32_t latency = timestamp - edge_us;
    latency_sum_us += latency;
    latency_max_us = latency > latency_max_us ? latency : latency_max_us;
    latency_count++;
}

bool wait_interrupts(uint16_t count)
{
    for (uint16_t i = 0; i < 200 && interrupts < count; i++) {
        hal.scheduler->delay_microseconds(100);
    }
    return interrupts >= count;
}

void test_output()
{
    hal.gpio->pinMode(pin, HAL_GPIO_OUTPUT);
    uint16_t errors = 0;
    const uint32_t t0 = AP_HAL::micros();
    for (uint16_t i = 0; i < 500; i++) {
        hal.gpio->write(pin, i & 1);
        if (hal.gpio->read(pin) != (i & 1)) {
            errors++;
        }
    }
    const uint32_t us = AP_HAL::micros() - t0;
    hal.console->printf("write then read back, 500 times: %u errors, %.1f us per write and read\n",
                        unsigned(errors), us / 500.0);

    errors = 0;
    hal.gpio->write(pin, 0);
    for (uint16_t i = 0; i < 100; i++) {
        hal.gpio->toggle(pin);
        if (hal.gpio->read(pin) != ((i + 1) & 1)) {
            errors++;
        }
    }
    hal.console->printf("toggle, 100 times: %u errors\n", unsigned(errors));

    AP_HAL::DigitalSource *src = hal.gpio->channel(pin);
    if (src == nullptr) {
        hal.console->printf("DigitalSource: none for pin %u\n", unsigned(pin));
        return;
    }
    errors = 0;
    src->mode(HAL_GPIO_OUTPUT);
    for (uint16_t i = 0; i < 100; i++) {
        src->write(i & 1);
        if (src->read() != (i & 1)) {
            errors++;
        }
    }
    src->write(0);
    hal.console->printf("DigitalSource write then read back, 100 times: %u errors\n",
                        unsigned(errors));
    // AP_HAL::DigitalSource has no virtual destructor: drivers keep theirs
}

void test_interrupts()
{
    hal.gpio->write(pin, 0);
    if (!hal.gpio->attach_interrupt(pin, FUNCTOR_BIND(&edge_counter, &EdgeCounter::on_edge, void, uint8_t, bool, uint32_t),
                                    AP_HAL::GPIO::INTERRUPT_BOTH)) {
        hal.console->printf("interrupts: not supported on pin %u\n", unsigned(pin));
        return;
    }
    uint16_t missed = 0;
    for (uint16_t i = 0; i < EDGES; i++) {
        const uint16_t before = interrupts;
        edge_us = AP_HAL::micros();
        hal.gpio->write(pin, (i + 1) & 1);
        if (!wait_interrupts(before + 1)) {
            missed++;
            // keep the count in step with the edges
            interrupts = before + 1;
        }
    }
    hal.console->printf("interrupts on both edges: %u edges, %u missed, %u with the wrong state; "
                        "latency mean %.1f max %u us\n",
                        unsigned(EDGES), unsigned(missed), unsigned(wrong_state),
                        latency_count ? double(latency_sum_us) / latency_count : 0.0,
                        unsigned(latency_max_us));

    hal.gpio->detach_interrupt(pin);
    const uint16_t before = interrupts;
    for (uint16_t i = 0; i < 20; i++) {
        hal.gpio->toggle(pin);
        hal.scheduler->delay_microseconds(200);
    }
    hal.scheduler->delay(10);
    hal.console->printf("after detaching: %u interrupts for 20 edges\n",
                        unsigned(interrupts - before));
}

}

void setup()
{
    hal.console->printf("GPIOTest on pin %u\n", unsigned(pin));
    if (!hal.gpio->valid_pin(pin)) {
        hal.console->printf("GPIOTest: pin %u is not a GPIO pin of this board\n", unsigned(pin));
        return;
    }
    test_output();
    test_interrupts();
    hal.gpio->write(pin, 0);
    hal.gpio->pinMode(pin, HAL_GPIO_INPUT);
    hal.console->printf("GPIOTest: done\n");
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
