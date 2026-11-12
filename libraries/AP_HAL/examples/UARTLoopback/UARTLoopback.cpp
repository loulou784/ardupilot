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
  UARTLoopback: check a board's serial ports, each with its TX joined to
  its own RX, through the AP_HAL interfaces, so that it runs on every HAL.

  The ports tested are SERIAL1 to SERIAL9 that have a device: on Linux
  and QNX, the ones given on the command line, eg.
      UARTLoopback --serial1 /dev/ser2 --serial2 /dev/ser3

  On each port, at each rate, 2 kB of a pattern particular to the port
  and rate are sent, and read back through the UART driver, as the
  vehicle code reads; the report gives the bytes missing, wrong, and the
  rate reached.  While a port sends, the other ports must receive
  nothing: bytes they receive are reported, as a jumper on the wrong
  pins would give.
*/
#include <string.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

constexpr uint16_t LENGTH = 2048;
constexpr uint16_t BUFFER = 8192;
constexpr uint8_t FIRST_PORT = 1;
constexpr uint8_t LAST_PORT = 9;

struct Rate {
    uint32_t baud;
    uint8_t parity;     // 0 none, 1 odd, 2 even
    uint8_t stop_bits;
    const char *use;
};

// the rates of the SERIALn_BAUD options up to 921600, and the SBUS and
// CRSF rates
const Rate rates[] = {
    {    9600, 0, 1, "" },
    {   57600, 0, 1, "" },
    {  100000, 2, 2, " (SBUS, 8E2)" },
    {  115200, 0, 1, "" },
    {  230400, 0, 1, "" },
    {  420000, 0, 1, " (CRSF)" },
    {  460800, 0, 1, "" },
    {  921600, 0, 1, "" },
};

uint8_t sent[LENGTH];
uint8_t received[LENGTH];

// a pattern with every byte value, different on each port and rate
void make_pattern(uint8_t port, uint8_t rate)
{
    uint32_t x = 0x9E3779B9U ^ (uint32_t(port) << 8) ^ rate;
    for (uint16_t i = 0; i < LENGTH; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        sent[i] = uint8_t(x);
    }
}

uint8_t bits_per_byte(const Rate &r)
{
    return 1 + 8 + (r.parity ? 1 : 0) + r.stop_bits;
}

void configure(AP_HAL::UARTDriver *uart, const Rate &r)
{
    uart->begin(r.baud, BUFFER, BUFFER);
    uart->configure_parity(r.parity);
    uart->set_stop_bits(r.stop_bits);
}

// read and count what a port that is not sending received
uint32_t drain(AP_HAL::UARTDriver *uart)
{
    uint32_t n = 0;
    uint8_t b[64];
    for (;;) {
        const ssize_t got = uart->read(b, sizeof(b));
        if (got <= 0) {
            return n;
        }
        n += got;
    }
}

struct Result {
    uint16_t received;
    uint16_t wrong;
    int32_t first_wrong;
    uint32_t first_byte_us;
    uint32_t total_us;
};

Result run(AP_HAL::UARTDriver *uart, const Rate &r)
{
    Result res {};
    res.first_wrong = -1;

    // the time the data takes on the wire, then as long again, and
    // 200 ms for the UART threads and the drivers
    const uint32_t wire_us = uint32_t(uint64_t(LENGTH) * bits_per_byte(r) * 1000000U / r.baud);
    const uint32_t timeout_us = 2 * wire_us + 200000;

    uint16_t written = 0;
    const uint32_t t0 = AP_HAL::micros();
    while (AP_HAL::micros() - t0 < timeout_us && res.received < LENGTH) {
        if (written < LENGTH) {
            const uint32_t space = uart->txspace();
            const uint16_t n = space < uint32_t(LENGTH - written) ? space : LENGTH - written;
            if (n > 0) {
                written += uart->write(&sent[written], n);
            }
        }
        const ssize_t got = uart->read(&received[res.received], LENGTH - res.received);
        if (got > 0) {
            if (res.received == 0) {
                res.first_byte_us = AP_HAL::micros() - t0;
            }
            res.received += got;
        } else {
            hal.scheduler->delay_microseconds(500);
        }
    }
    res.total_us = AP_HAL::micros() - t0;
    for (uint16_t i = 0; i < res.received; i++) {
        if (received[i] != sent[i]) {
            res.wrong++;
            if (res.first_wrong < 0) {
                res.first_wrong = i;
            }
        }
    }
    // more than was sent: the rest is an error too
    res.wrong += drain(uart);
    return res;
}

void test_port(uint8_t port, uint32_t &failures)
{
    AP_HAL::UARTDriver *uart = hal.serial(port);
    hal.console->printf("SERIAL%u:\n", unsigned(port));
    for (uint8_t ri = 0; ri < ARRAY_SIZE(rates); ri++) {
        const Rate &r = rates[ri];
        configure(uart, r);
        // the other ports at the same rate, to see what they pick up
        for (uint8_t p = FIRST_PORT; p <= LAST_PORT; p++) {
            AP_HAL::UARTDriver *other = hal.serial(p);
            if (p != port && other != nullptr && other->is_initialized()) {
                configure(other, r);
            }
        }
        hal.scheduler->delay(50);
        for (uint8_t p = FIRST_PORT; p <= LAST_PORT; p++) {
            AP_HAL::UARTDriver *u = hal.serial(p);
            if (u != nullptr && u->is_initialized()) {
                drain(u);
            }
        }
        make_pattern(port, ri);
        const Result res = run(uart, r);

        hal.scheduler->delay(20);
        char strays[64] = "";
        size_t len = 0;
        for (uint8_t p = FIRST_PORT; p <= LAST_PORT; p++) {
            AP_HAL::UARTDriver *other = hal.serial(p);
            if (p == port || other == nullptr || !other->is_initialized()) {
                continue;
            }
            const uint32_t n = drain(other);
            if (n > 0 && len < sizeof(strays)) {
                len += hal.util->snprintf(&strays[len], sizeof(strays) - len,
                                          ", %u on SERIAL%u", unsigned(n), unsigned(p));
            }
        }

        const bool ok = res.received == LENGTH && res.wrong == 0 && len == 0;
        if (!ok) {
            failures++;
        }
        // the time all the data took to come back, against the time it
        // takes on the wire
        const uint32_t wire_us = uint32_t(uint64_t(LENGTH) * bits_per_byte(r) * 1000000U / r.baud);
        hal.console->printf("  %7u%s: %s, ", unsigned(r.baud), r.use, ok ? "ok" : "FAILED");
        if (res.received == 0) {
            hal.console->printf("nothing came back%s\n", strays);
            continue;
        }
        hal.console->printf("%u of %u bytes back, %u wrong",
                            unsigned(res.received), unsigned(LENGTH), unsigned(res.wrong));
        if (res.first_wrong >= 0) {
            hal.console->printf(" (first at byte %d)", int(res.first_wrong));
        }
        hal.console->printf("%s; first byte back after %.1f ms, all after %.1f ms (%.1f ms on the wire)\n",
                            strays, res.first_byte_us * 1e-3, res.total_us * 1e-3, wire_us * 1e-3);
    }
    configure(uart, rates[3]);
    uart->configure_parity(0);
    uart->set_stop_bits(1);
}

}

void setup()
{
    hal.scheduler->delay(200);
    // a port with a device is initialised by begin(); the others are not
    uint8_t ports = 0;
    for (uint8_t p = FIRST_PORT; p <= LAST_PORT; p++) {
        AP_HAL::UARTDriver *uart = hal.serial(p);
        if (uart != nullptr) {
            uart->begin(115200, BUFFER, BUFFER);
            if (uart->is_initialized()) {
                ports++;
            }
        }
    }
    hal.console->printf("UARTLoopback: %u ports, each with its TX joined to its RX\n",
                        unsigned(ports));
    if (ports == 0) {
        hal.console->printf("UARTLoopback: no serial port has a device; give them on the "
                            "command line, eg. --serial1 /dev/ser2\n");
        return;
    }
    uint32_t failures = 0;
    for (uint8_t p = FIRST_PORT; p <= LAST_PORT; p++) {
        AP_HAL::UARTDriver *uart = hal.serial(p);
        if (uart != nullptr && uart->is_initialized()) {
            test_port(p, failures);
        }
    }
    hal.console->printf("UARTLoopback: done, %u failed\n", unsigned(failures));
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
