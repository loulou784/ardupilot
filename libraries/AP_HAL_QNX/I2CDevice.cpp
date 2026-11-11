/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  I2C devices, ported from AP_HAL_Linux.

  QNX difference: QNX has no i2c-dev.  Each I2C controller has a resource
  manager, eg. i2c-bcm2711 on the Raspberry Pi 4, at /dev/i2c<n>, and a
  transfer is one devctl(): DCMD_I2C_SEND, DCMD_I2C_RECV, or
  DCMD_I2C_SENDRECV for a write followed by a read.  The data goes with
  the message, from and to the caller's buffers, without copies.
 */
#include "I2CDevice.h"

#include <devctl.h>
#include <errno.h>
#include <fcntl.h>
#include <hw/i2c.h>
#include <stdio.h>
#include <string.h>
#include <sys/uio.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "DeviceBus.h"

extern const AP_HAL::HAL& hal;

namespace QNX {

/* Private struct to maintain for each bus */
class I2CBus : public DeviceBus {
public:
    I2CBus(uint8_t bus_)
        : DeviceBus(AP_HAL::Device::BUS_TYPE_I2C, bus_)
        , bus(bus_)
    {
    }

    ~I2CBus();

    int open();

    // lower the bus clock to clock_hz, if it is lower than the current one
    void lower_clock(uint32_t clock_hz, uint8_t address);

    // one transfer: write, read, or write then read
    int transfer(uint8_t address, const uint8_t *send, uint32_t send_len,
                 uint8_t *recv, uint32_t recv_len);

    int fd = -1;
    const uint8_t bus;

    // the boot image starts the I2C drivers at 400 kHz, the fastest that
    // the Raspberry Pi 4 driver supports
    uint32_t clock_hz = 400000;
};

I2CBus::~I2CBus()
{
    teardown();
    if (fd >= 0) {
        ::close(fd);
    }
}

int I2CBus::open()
{
    char path[sizeof("/dev/i2cXXX")];
    int r;

    if (fd >= 0) {
        return -EBUSY;
    }

    // QNX names I2C buses /dev/i2c<n>, where Linux has /dev/i2c-<n>
    r = snprintf(path, sizeof(path), "/dev/i2c%u", bus);
    if (r < 0 || r >= (int)sizeof(path)) {
        return -EINVAL;
    }

    fd = ::open(path, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        return -errno;
    }

    return fd;
}

void I2CBus::lower_clock(uint32_t new_clock_hz, uint8_t address)
{
    /*
      QNX difference: the bus clock belongs to the bus, not to the device,
      as on ChibiOS.  Linux leaves it to the device tree.  As ChibiOS
      does, run the whole bus at the speed of its slowest device; only
      devices set up by parameters, such as some battery monitors, ask
      for less than 400 kHz.
     */
    if (new_clock_hz == 0 || new_clock_hz >= clock_hz) {
        return;
    }
    unsigned speed = new_clock_hz;
    const int r = devctl(fd, DCMD_I2C_SET_BUS_SPEED, &speed, sizeof(speed), nullptr);
    if (r != EOK) {
        hal.console->printf("I2C: failed to set bus %u to %u kHz for device 0x%02x: %s\n",
                            unsigned(bus), unsigned(new_clock_hz / 1000), unsigned(address),
                            strerror(r));
        return;
    }
    hal.console->printf("I2C: bus %u now runs at %u kHz, for device 0x%02x\n",
                        unsigned(bus), unsigned(new_clock_hz / 1000), unsigned(address));
    clock_hz = new_clock_hz;
}

int I2CBus::transfer(uint8_t address, const uint8_t *send, uint32_t send_len,
                     uint8_t *recv, uint32_t recv_len)
{
    const i2c_addr_t slave { address, I2C_ADDRFMT_7BIT };
    iov_t siov[2], riov[2];

    // devctlv() returns the errno value itself
    if (send_len != 0 && recv_len != 0) {
        // a write then a read, with a stop at the end
        i2c_sendrecv_t hdr {};
        hdr.slave = slave;
        hdr.send_len = send_len;
        hdr.recv_len = recv_len;
        hdr.stop = 1;
        SETIOV(&siov[0], &hdr, sizeof(hdr));
        SETIOV(&siov[1], const_cast<uint8_t*>(send), send_len);
        SETIOV(&riov[0], &hdr, sizeof(hdr));
        SETIOV(&riov[1], recv, recv_len);
        return devctlv(fd, DCMD_I2C_SENDRECV, 2, 2, siov, riov, nullptr);
    }
    if (send_len != 0) {
        i2c_send_t hdr {};
        hdr.slave = slave;
        hdr.len = send_len;
        hdr.stop = 1;
        SETIOV(&siov[0], &hdr, sizeof(hdr));
        SETIOV(&siov[1], const_cast<uint8_t*>(send), send_len);
        return devctlv(fd, DCMD_I2C_SEND, 2, 0, siov, nullptr, nullptr);
    }
    i2c_recv_t hdr {};
    hdr.slave = slave;
    hdr.len = recv_len;
    hdr.stop = 1;
    SETIOV(&siov[0], &hdr, sizeof(hdr));
    SETIOV(&riov[0], &hdr, sizeof(hdr));
    SETIOV(&riov[1], recv, recv_len);
    return devctlv(fd, DCMD_I2C_RECV, 1, 2, siov, riov, nullptr);
}

I2CDevice::I2CDevice(I2CBus &bus, uint8_t address)
    : _bus(bus)
    , _address(address)
{
    set_device_bus(bus.bus);
    set_device_address(address);
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (_split_transfers && send_len > 0 && recv_len > 0) {
        return transfer(send, send_len, nullptr, 0) &&
            transfer(nullptr, 0, recv, recv_len);
    }

    if (!send) {
        send_len = 0;
    }
    if (!recv) {
        recv_len = 0;
    }

    /* interpret it as an input error if nothing has to be done */
    if (send_len == 0 && recv_len == 0) {
        return false;
    }

    /* the most the QNX I2C drivers take in one transfer */
    if (send_len > MAX_I2C_DATA_LEN || recv_len > MAX_I2C_DATA_LEN) {
        return false;
    }

    /*
      Errors are not printed: absent devices do not acknowledge, and
      probing for them is normal.  They are EIO, from the driver, or
      ETIMEDOUT, eg. when a device stretches the clock for too long.
     */
    int r;
    unsigned retries = _retries;
    do {
        r = _bus.transfer(_address, send, send_len, recv, recv_len);
    } while (r != EOK && retries-- > 0);

    return r == EOK;
}

bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    /*
      QNX difference: Linux sends all the reads as one I2C_RDWR ioctl;
      QNX I2C drivers take one write and read per devctl(), so this is a
      devctl() per read.
     */
    first_reg |= _read_flag;

    while (times > 0) {
        if (!transfer(&first_reg, 1, recv, recv_len)) {
            return false;
        }
        recv += recv_len;
        times--;
    }

    return true;
}

AP_HAL::Semaphore *I2CDevice::get_semaphore()
{
    return &_bus.semaphore();
}

AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _bus.register_periodic_callback(period_usec, cb);
}

bool I2CDevice::adjust_periodic_callback(
    AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return _bus.adjust_periodic_callback(h, period_usec);
}

bool I2CDevice::unregister_callback(AP_HAL::Device::PeriodicHandle h)
{
    return _bus.unregister_callback(h);
}

AP_HAL::I2CDevice *
I2CDeviceManager::get_device_ptr(uint8_t bus, uint8_t address,
                                 uint32_t bus_clock,
                                 bool use_smbus,
                                 uint32_t timeout_ms)
{
    if (bus >= ARRAY_SIZE(_buses)) {
        return nullptr;
    }

    WITH_SEMAPHORE(_buses_semaphore);

    if (_buses[bus] == nullptr) {
        /* Bus not open yet: open it */
        I2CBus *b = NEW_NOTHROW I2CBus(bus);
        if (!b) {
            return nullptr;
        }
        if (b->open() < 0) {
            /* as on Linux, a bus that does not exist has no devices */
            delete b;
            return nullptr;
        }
        _buses[bus] = b;
    }

    I2CBus &b = *_buses[bus];
    auto *dev = NEW_NOTHROW I2CDevice(b, address);
    if (!dev) {
        return nullptr;
    }

    // with the bus semaphore, as the bus thread may be using the bus
    {
        WITH_SEMAPHORE(b.semaphore());
        b.lower_clock(bus_clock, address);
    }

    return dev;
}

void I2CDeviceManager::teardown()
{
    for (I2CBus *b : _buses) {
        if (b != nullptr) {
            b->teardown();
        }
    }
}

/*
  get mask of bus numbers for all configured I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask(void) const
{
    return HAL_QNX_I2C_BUS_MASK;
}

/*
  get mask of bus numbers for all configured internal I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
    return HAL_QNX_I2C_INTERNAL_BUS_MASK;
}

/*
  get mask of bus numbers for all configured external I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
    return HAL_QNX_I2C_EXTERNAL_BUS_MASK;
}

}
