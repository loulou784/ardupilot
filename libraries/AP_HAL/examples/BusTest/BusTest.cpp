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
  BusTest: check a board's I2C and SPI buses using only the AP_HAL
  interfaces, so that it runs on every HAL:

  - scan every I2C bus in the bus mask, reading one byte from each
    address, and list the devices that answer, with the time a probe takes
  - read the first register and the usual WHO_AM_I registers of every SPI
    device in the board's table, with the time a transfer takes
  - run periodic callbacks on the bus threads, and measure how regularly
    they run, with a transfer in each callback

  The scan only reads, as i2cdetect -r does, which is safe for nearly all
  devices.
*/
#include <string.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

constexpr uint8_t MAX_I2C_BUS = 32;
constexpr uint32_t CALLBACK_SAMPLES = 2000;

// percentile of the samples, in us, from a 1 us histogram up to 20 ms
uint32_t percentile(const uint32_t *samples, uint32_t n, float pct)
{
    static constexpr uint32_t BINS = 20000;
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

// a periodic callback that records its periods and does one transfer
class CallbackTimer {
public:
    CallbackTimer(const char *name, uint32_t period_us)
        : _name(name), _period_us(period_us) { }

    bool start(AP_HAL::Device *dev, uint8_t send_len, uint8_t recv_len) {
        _dev = dev;
        _send_len = send_len;
        _recv_len = recv_len;
        return dev->register_periodic_callback(
            _period_us, FUNCTOR_BIND_MEMBER(&CallbackTimer::tick, void)) != nullptr;
    }

    bool done() const { return _count >= CALLBACK_SAMPLES; }

    void report() const {
        uint64_t sum = 0;
        uint32_t min = UINT32_MAX, max = 0;
        for (uint32_t i = 0; i < _count; i++) {
            sum += _period[i];
            min = _period[i] < min ? _period[i] : min;
            max = _period[i] > max ? _period[i] : max;
        }
        hal.console->printf("%s every %u us (%u samples): mean %.1f min %u max %u us; "
                            "transfer mean %.1f us, %u failed\n",
                            _name, unsigned(_period_us), unsigned(_count),
                            _count ? double(sum) / _count : 0.0, unsigned(min), unsigned(max),
                            _count ? double(_transfer_sum_us) / _count : 0.0,
                            unsigned(_transfer_failures));
        hal.console->printf("%s period percentiles: 0.1%% %u, 1%% %u, 50%% %u, 99%% %u, 99.9%% %u us\n",
                            _name,
                            unsigned(percentile(_period, _count, 0.1f)),
                            unsigned(percentile(_period, _count, 1.0f)),
                            unsigned(percentile(_period, _count, 50.0f)),
                            unsigned(percentile(_period, _count, 99.0f)),
                            unsigned(percentile(_period, _count, 99.9f)));
    }

private:
    void tick() {
        const uint32_t now = AP_HAL::micros();
        if (_last_us != 0 && _count < CALLBACK_SAMPLES) {
            _period[_count++] = now - _last_us;
        }
        _last_us = now;
        if (_count >= CALLBACK_SAMPLES) {
            return;
        }
        uint8_t send[2] {}, recv[8];
        const uint32_t t0 = AP_HAL::micros();
        if (!_dev->transfer(send, _send_len, recv, _recv_len)) {
            _transfer_failures++;
        }
        _transfer_sum_us += AP_HAL::micros() - t0;
    }

    const char *_name;
    const uint32_t _period_us;
    AP_HAL::Device *_dev;
    uint8_t _send_len;
    uint8_t _recv_len;
    uint32_t _last_us;
    uint32_t _period[CALLBACK_SAMPLES];
    volatile uint32_t _count;
    uint64_t _transfer_sum_us;
    uint32_t _transfer_failures;
};

CallbackTimer *timers[3];
uint8_t num_timers;

void scan_i2c()
{
    const uint32_t mask = hal.i2c_mgr->get_bus_mask();
    for (uint8_t bus = 0; bus < MAX_I2C_BUS; bus++) {
        if (!(mask & (1U << bus))) {
            continue;
        }
        // a bus that does not exist gives no devices
        AP_HAL::I2CDevice *probe = hal.i2c_mgr->get_device_ptr(bus, 0x08);
        if (probe == nullptr) {
            continue;
        }
        delete probe;

        hal.console->printf("I2C bus %u:", unsigned(bus));
        uint32_t found = 0, probes = 0;
        uint64_t probe_us = 0;
        for (uint8_t address = 0x08; address < 0x78; address++) {
            AP_HAL::I2CDevice *dev = hal.i2c_mgr->get_device_ptr(bus, address);
            if (dev == nullptr) {
                break;
            }
            uint8_t b;
            bool ok;
            {
                WITH_SEMAPHORE(dev->get_semaphore());
                const uint32_t t0 = AP_HAL::micros();
                ok = dev->transfer(nullptr, 0, &b, 1);
                probe_us += AP_HAL::micros() - t0;
                probes++;
            }
            if (ok) {
                hal.console->printf(" 0x%02x", unsigned(address));
                found++;
            }
            delete dev;
        }
        hal.console->printf("%s; %u devices, %.1f us per probe\n",
                            found ? "" : " none", unsigned(found),
                            probes ? double(probe_us) / probes : 0.0);
    }
}

void read_spi()
{
    const uint8_t count = hal.spi->get_count();
    if (count == 0) {
        hal.console->printf("SPI: no devices in this board's table\n");
        return;
    }
    for (uint8_t i = 0; i < count; i++) {
        const char *name = hal.spi->get_device_name(i);
        if (name == nullptr) {
            continue;
        }
        AP_HAL::SPIDevice *dev = hal.spi->get_device_ptr(name);
        if (dev == nullptr) {
            hal.console->printf("SPI %s: not available\n", name);
            continue;
        }
        // register 0, and WHO_AM_I of InvenSense (0x75), ST and Bosch
        // (0x0f, 0x00) devices, with the usual read flag
        static const uint8_t regs[] = { 0x00, 0x0f, 0x75 };
        uint8_t values[ARRAY_SIZE(regs)] {};
        bool ok = true;
        uint32_t us = 0;
        {
            WITH_SEMAPHORE(dev->get_semaphore());
            dev->set_speed(AP_HAL::Device::SPEED_LOW);
            for (uint8_t r = 0; r < ARRAY_SIZE(regs); r++) {
                const uint8_t reg = regs[r] | 0x80;
                const uint32_t t0 = AP_HAL::micros();
                ok = dev->transfer(&reg, 1, &values[r], 1) && ok;
                us += AP_HAL::micros() - t0;
            }
            dev->set_speed(AP_HAL::Device::SPEED_HIGH);
        }
        hal.console->printf("SPI %s (bus %u device %u): %s, reg 0x00 0x%02x, 0x0f 0x%02x, "
                            "0x75 0x%02x; %.1f us per transfer\n",
                            name, unsigned(dev->bus_num()), unsigned(dev->get_bus_address()),
                            ok ? "ok" : "transfer failed", unsigned(values[0]),
                            unsigned(values[1]), unsigned(values[2]),
                            double(us) / ARRAY_SIZE(regs));
        if (num_timers < ARRAY_SIZE(timers) && timers[num_timers] == nullptr) {
            timers[num_timers] = NEW_NOTHROW CallbackTimer("SPI callback", 1000);
            if (timers[num_timers] && timers[num_timers]->start(dev, 2, 0)) {
                num_timers++;
            }
        }
        // the device stays, for its callback
    }
}

void start_i2c_callbacks()
{
    // two callbacks on the first I2C bus, each reading from an address
    // that is normally free, so that the transfers fail quickly
    const uint32_t mask = hal.i2c_mgr->get_bus_mask();
    for (uint8_t bus = 0; bus < MAX_I2C_BUS; bus++) {
        if (!(mask & (1U << bus))) {
            continue;
        }
        AP_HAL::I2CDevice *dev = hal.i2c_mgr->get_device_ptr(bus, 0x08);
        if (dev == nullptr) {
            continue;
        }
        static const struct {
            const char *name;
            uint32_t period_us;
        } callbacks[] = {
            { "I2C callback 1", 1000 },
            { "I2C callback 2", 2500 },
        };
        for (const auto &c : callbacks) {
            if (num_timers < ARRAY_SIZE(timers)) {
                timers[num_timers] = NEW_NOTHROW CallbackTimer(c.name, c.period_us);
                if (timers[num_timers] && timers[num_timers]->start(dev, 1, 1)) {
                    num_timers++;
                }
            }
        }
        return;
    }
}

}

void setup()
{
    hal.console->printf("BusTest: I2C bus mask 0x%08x\n",
                        unsigned(hal.i2c_mgr->get_bus_mask()));
    hal.scheduler->delay(100);
    scan_i2c();
    read_spi();
    start_i2c_callbacks();
    if (num_timers == 0) {
        hal.console->printf("BusTest: no bus for the callback test\n");
    }
}

void loop()
{
    static bool done;
    if (done) {
        hal.scheduler->delay(1000);
        return;
    }
    bool all_done = true;
    for (uint8_t i = 0; i < num_timers; i++) {
        all_done = all_done && timers[i]->done();
    }
    if (!all_done) {
        hal.scheduler->delay(100);
        return;
    }
    for (uint8_t i = 0; i < num_timers; i++) {
        timers[i]->report();
    }
    hal.console->printf("BusTest: done\n");
    done = true;
}

AP_HAL_MAIN();
