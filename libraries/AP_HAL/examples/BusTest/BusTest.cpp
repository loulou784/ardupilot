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
  - read the first register and the usual WHO_AM_I registers of every I2C
    device found and every SPI device in the board's table, with the time
    a transfer takes
  - run periodic callbacks on the bus threads, and measure how regularly
    they run, with a read in each callback: WHO_AM_I of the SPI devices,
    checked against the first value read, and a free I2C address

  Nothing is written to a device: the scan only reads, as i2cdetect -r
  does, which is safe for nearly all devices, and the register reads
  send only the register address.
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

// a periodic callback that records its periods and does one transfer: a
// read, which sends at most one byte (a register address), so that the
// test never writes to a device; the bytes read are compared with the
// first ones read
class CallbackTimer {
public:
    CallbackTimer(const char *name, uint32_t period_us)
        : _name(name), _period_us(period_us) { }

    bool start(AP_HAL::Device *dev, const uint8_t *send, uint8_t send_len, uint8_t recv_len) {
        if (send_len > sizeof(_send) || recv_len > sizeof(_first)) {
            return false;
        }
        _dev = dev;
        if (send_len > 0) {
            memcpy(_send, send, send_len);
        }
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
        if (_have_first && _recv_len > 0) {
            hal.console->printf("%s read 0x%02x first, then other data %u times\n",
                                _name, unsigned(_first[0]), unsigned(_mismatches));
        }
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
        uint8_t recv[sizeof(_first)];
        const uint32_t t0 = AP_HAL::micros();
        const bool ok = _dev->transfer(_send, _send_len, recv, _recv_len);
        _transfer_sum_us += AP_HAL::micros() - t0;
        if (!ok) {
            _transfer_failures++;
        } else if (!_have_first) {
            memcpy(_first, recv, _recv_len);
            _have_first = true;
        } else if (memcmp(_first, recv, _recv_len) != 0) {
            _mismatches++;
        }
    }

    const char *_name;
    const uint32_t _period_us;
    AP_HAL::Device *_dev;
    uint8_t _send[1];
    uint8_t _send_len;
    uint8_t _recv_len;
    uint8_t _first[8];
    bool _have_first;
    uint32_t _mismatches;
    uint32_t _last_us;
    uint32_t _period[CALLBACK_SAMPLES];
    volatile uint32_t _count;
    uint64_t _transfer_sum_us;
    uint32_t _transfer_failures;
};

CallbackTimer *timers[8];
uint8_t num_timers;

// register 0, and the WHO_AM_I registers of InvenSense (0x75), ST (0x0f)
// and Bosch (0x00) devices
const uint8_t id_regs[] = { 0x00, 0x0f, 0x75 };

// read the ID registers of a device that answered the scan: each read is
// a register write then a read, with a repeated start
void read_i2c_ids(uint8_t bus, uint8_t address)
{
    AP_HAL::I2CDevice *dev = hal.i2c_mgr->get_device_ptr(bus, address);
    if (dev == nullptr) {
        return;
    }
    uint8_t values[ARRAY_SIZE(id_regs)] {};
    uint8_t failed = 0;
    uint32_t us = 0;
    {
        WITH_SEMAPHORE(dev->get_semaphore());
        for (uint8_t r = 0; r < ARRAY_SIZE(id_regs); r++) {
            const uint32_t t0 = AP_HAL::micros();
            if (!dev->read_registers(id_regs[r], &values[r], 1)) {
                failed++;
            }
            us += AP_HAL::micros() - t0;
        }
    }
    hal.console->printf("  0x%02x: reg 0x00 0x%02x, 0x0f 0x%02x, 0x75 0x%02x; %u of %u reads failed, "
                        "%.1f us per read\n",
                        unsigned(address), unsigned(values[0]), unsigned(values[1]),
                        unsigned(values[2]), unsigned(failed), unsigned(ARRAY_SIZE(id_regs)),
                        double(us) / ARRAY_SIZE(id_regs));
    delete dev;
}

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
        uint8_t addresses[0x78];
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
                addresses[found++] = address;
            }
            delete dev;
        }
        hal.console->printf("%s; %u devices, %.1f us per probe\n",
                            found ? "" : " none", unsigned(found),
                            probes ? double(probe_us) / probes : 0.0);
        for (uint32_t i = 0; i < found; i++) {
            read_i2c_ids(bus, addresses[i]);
        }
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
        // the ID registers, with the usual read flag
        const uint8_t *regs = id_regs;
        uint8_t values[ARRAY_SIZE(id_regs)] {};
        bool ok = true;
        uint32_t us = 0;
        {
            WITH_SEMAPHORE(dev->get_semaphore());
            dev->set_speed(AP_HAL::Device::SPEED_LOW);
            for (uint8_t r = 0; r < ARRAY_SIZE(id_regs); r++) {
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
                            double(us) / ARRAY_SIZE(id_regs));
        if (num_timers < ARRAY_SIZE(timers) && timers[num_timers] == nullptr) {
            // the callback reads WHO_AM_I at the low speed: the InvenSense
            // MPU chips allow only 1 MHz for registers other than the
            // sensor data
            {
                WITH_SEMAPHORE(dev->get_semaphore());
                dev->set_speed(AP_HAL::Device::SPEED_LOW);
            }
            static const uint8_t who_am_i = 0x75 | 0x80;
            timers[num_timers] = NEW_NOTHROW CallbackTimer("SPI callback", 1000);
            if (timers[num_timers] && timers[num_timers]->start(dev, &who_am_i, 1, 1)) {
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
                if (timers[num_timers] && timers[num_timers]->start(dev, nullptr, 0, 1)) {
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
