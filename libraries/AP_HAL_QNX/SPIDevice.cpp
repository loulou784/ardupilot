/*
 * Copyright (C) 2015  Intel Corporation. All rights reserved.
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
  SPI devices, ported from AP_HAL_Linux.

  QNX difference: QNX has no spidev.  io-spi drivers, eg. spi-bcm2711 on
  the Raspberry Pi 4, have a node for each chip select, named in their
  configuration file, and an exchange is one devctl(), DCMD_SPI_DATA_XCHNG,
  which sends and receives the same number of bytes with the chip select
  held, from and to the caller's buffers without copies.  Mode and clock
  are set on the node with DCMD_SPI_SET_CONFIG, only when they change.
 */
#include "SPIDevice.h"

#include <devctl.h>
#include <errno.h>
#include <fcntl.h>
#include <hw/io-spi.h>
#include <stdio.h>
#include <string.h>
#include <sys/uio.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "DeviceBus.h"
#include "GPIO.h"

extern const AP_HAL::HAL& hal;

namespace QNX {

#define MHZ (1000U*1000U)
#define KHZ (1000U)
#define SPI_CS_KERNEL -1

/* SPI modes, as Linux's spidev defines them, in io-spi mode bits */
#define SPI_MODE_0 (SPI_MODE_CPOL_0 | SPI_MODE_CPHA_0)
#define SPI_MODE_1 (SPI_MODE_CPOL_0 | SPI_MODE_CPHA_1)
#define SPI_MODE_2 (SPI_MODE_CPOL_1 | SPI_MODE_CPHA_0)
#define SPI_MODE_3 (SPI_MODE_CPOL_1 | SPI_MODE_CPHA_1)

#ifdef HAL_SPI_DEVICE_LIST
SPIDesc SPIDeviceManager::_device[] = {
    HAL_SPI_DEVICE_LIST
};
#define QNX_SPI_DEVICE_NUM_DEVICES ARRAY_SIZE(SPIDeviceManager::_device)
#else
// empty device table
SPIDesc SPIDeviceManager::_device[] = {
    SPIDesc("**dummy**", 0, 0, "", SPI_MODE_3, 0, 0, 0 * MHZ, 0 * MHZ),
};
#define QNX_SPI_DEVICE_NUM_DEVICES 0
#endif

const uint8_t SPIDeviceManager::_n_device_desc = QNX_SPI_DEVICE_NUM_DEVICES;

// the longest exchange io-spi takes
static constexpr uint32_t MAX_EXCHANGE = SPI_BUF_MAX_64K;

// a node (chip select) of a bus, and the configuration last set on it
struct SPINode {
    const char *path = nullptr;
    int fd = -1;
    bool configured = false;
    uint32_t mode = 0;
    uint32_t clock_hz = 0;
};

/* Private struct to maintain for each bus */
class SPIBus : public DeviceBus {
public:
    SPIBus(uint16_t bus_)
        : DeviceBus(AP_HAL::Device::BUS_TYPE_SPI, bus_)
        , bus(bus_)
    {
    }

    ~SPIBus();

    // the node's file descriptor, opened on first use; -1 on failure
    int open(const char *path);

    // set mode and clock of the node, if not set already
    bool configure(int fd, uint32_t mode, uint32_t clock_hz);

    // zeros to send, and room for bytes to drop, len bytes of each
    bool scratch(uint32_t len);

    const uint16_t bus;
    uint8_t *zeros = nullptr;
    uint8_t *drop = nullptr;

private:
    SPINode *_node(int fd);

    static constexpr uint8_t MAX_NODES = 8;
    SPINode _nodes[MAX_NODES];
    uint8_t _num_nodes = 0;
    uint32_t _scratch_len = 0;

    // the clock the controller was last set to; 0 if not known
    uint32_t _clock_hz = 0;
};

SPIBus::~SPIBus()
{
    teardown();
    for (uint8_t i = 0; i < _num_nodes; i++) {
        ::close(_nodes[i].fd);
    }
    delete[] zeros;
    delete[] drop;
}

int SPIBus::open(const char *path)
{
    for (uint8_t i = 0; i < _num_nodes; i++) {
        if (strcmp(_nodes[i].path, path) == 0) {
            /* Already open by another device */
            return _nodes[i].fd;
        }
    }
    if (_num_nodes >= MAX_NODES) {
        hal.console->printf("SPI: too many devices on bus %u for %s\n", unsigned(bus), path);
        return -1;
    }
    const int fd = ::open(path, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        /*
          QNX difference: Linux panics; here the device is not created,
          so its driver reports it missing, as for a device that does
          not answer
         */
        hal.console->printf("SPI: unable to open %s: %s\n", path, strerror(errno));
        return -1;
    }
    SPINode &n = _nodes[_num_nodes++];
    n.path = path;
    n.fd = fd;
    n.configured = false;
    return fd;
}

SPINode *SPIBus::_node(int fd)
{
    for (uint8_t i = 0; i < _num_nodes; i++) {
        if (_nodes[i].fd == fd) {
            return &_nodes[i];
        }
    }
    return nullptr;
}

bool SPIBus::configure(int fd, uint32_t mode, uint32_t clock_hz)
{
    SPINode *n = _node(fd);
    if (n == nullptr) {
        return false;
    }
    const bool node_set = n->configured && n->mode == mode && n->clock_hz == clock_hz;
    if (node_set && _clock_hz == clock_hz) {
        return true;
    }

    spi_cfg_t cfg {};
    cfg.mode = mode;
    cfg.clock_rate = clock_hz;
    int r;

    /*
      The controller has one clock for all its chip selects, and the
      Raspberry Pi 4 driver, spi-bcm2711, sets it only when the rate of
      the node being configured changes.  So when another node, or the
      driver's own start-up configuration, set the clock last, and this
      node's rate may not change, first set another rate, so that the
      driver sets the clock again.
     */
    if (_clock_hz != clock_hz && (!n->configured || n->clock_hz == clock_hz)) {
        spi_cfg_t other = cfg;
        other.clock_rate = clock_hz > 1 * MHZ ? clock_hz / 2 : clock_hz * 2;
        r = devctl(fd, DCMD_SPI_SET_CONFIG, &other, sizeof(other), nullptr);
        if (r != EOK) {
            hal.console->printf("SPI: failed to configure %s: %s\n", n->path, strerror(r));
            n->configured = false;
            _clock_hz = 0;
            return false;
        }
    }

    // devctl() returns the errno value itself
    r = devctl(fd, DCMD_SPI_SET_CONFIG, &cfg, sizeof(cfg), nullptr);
    if (r != EOK) {
        hal.console->printf("SPI: failed to configure %s: %s\n", n->path, strerror(r));
        n->configured = false;
        _clock_hz = 0;
        return false;
    }
    n->configured = true;
    n->mode = mode;
    n->clock_hz = clock_hz;
    _clock_hz = clock_hz;
    return true;
}

bool SPIBus::scratch(uint32_t len)
{
    if (len <= _scratch_len) {
        return true;
    }
    // grows to the longest transfer, normally while drivers start
    uint8_t *new_zeros = NEW_NOTHROW uint8_t[len];
    uint8_t *new_drop = NEW_NOTHROW uint8_t[len];
    if (new_zeros == nullptr || new_drop == nullptr) {
        delete[] new_zeros;
        delete[] new_drop;
        return false;
    }
    memset(new_zeros, 0, len);
    delete[] zeros;
    delete[] drop;
    zeros = new_zeros;
    drop = new_drop;
    _scratch_len = len;
    return true;
}

SPIDevice::SPIDevice(SPIBus &bus, SPIDesc &device_desc, int fd)
    : _bus(bus)
    , _desc(device_desc)
    , _fd(fd)
{
    set_device_bus(_bus.bus);
    set_device_address(_desc.subdev);
    _speed = _desc.highspeed;

    if (_desc.cs_pin != SPI_CS_KERNEL) {
        _cs = hal.gpio->channel(_desc.cs_pin);
        if (!_cs) {
            AP_HAL::panic("Unable to instantiate cs pin");
        }

        _cs->mode(HAL_GPIO_OUTPUT);

        // do not hold the SPI bus initially
        _cs_release();
    }
}

SPIDevice::~SPIDevice()
{
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed speed)
{
    switch (speed) {
    case AP_HAL::Device::SPEED_HIGH:
        _speed = _desc.highspeed;
        break;
    case AP_HAL::Device::SPEED_LOW:
        _speed = _desc.lowspeed;
        break;
    }

    return true;
}

bool SPIDevice::_exchange(const uint8_t *send, uint32_t send_len,
                          uint8_t *recv, uint32_t skip, uint32_t len)
{
    if (len == 0 || len > MAX_EXCHANGE) {
        return false;
    }

    /*
      With the bus semaphore, which device drivers should hold already:
      this keeps the node configuration and the scratch buffers of the
      bus consistent even if one does not.
     */
    WITH_SEMAPHORE(_bus.semaphore());

    const uint32_t mode = _desc.mode | SPI_MODE_BODER_MSB |
        (_desc.bits_per_word & SPI_MODE_WORD_WIDTH_MASK);
    if (!_bus.configure(_fd, mode, _speed)) {
        return false;
    }
    // a reply shorter than the exchange is cut by the kernel, so the
    // bytes received while writing need room only when a read follows
    const bool drop_some = skip > 0 && skip < len;
    if ((send_len < len || drop_some) && !_bus.scratch(len)) {
        return false;
    }

    spi_xchng_t hdr {};
    hdr.nbytes = len;
    iov_t siov[3], riov[3];
    int sparts = 0, rparts = 0;
    SETIOV(&siov[sparts++], &hdr, sizeof(hdr));
    if (send_len > 0) {
        SETIOV(&siov[sparts++], const_cast<uint8_t*>(send), send_len);
    }
    if (send_len < len) {
        SETIOV(&siov[sparts++], _bus.zeros, len - send_len);
    }
    SETIOV(&riov[rparts++], &hdr, sizeof(hdr));
    if (drop_some) {
        SETIOV(&riov[rparts++], _bus.drop, skip);
    }
    if (skip < len) {
        SETIOV(&riov[rparts++], recv, len - skip);
    }

    _cs_assert();
    // devctlv() returns the errno value itself
    const int r = devctlv(_fd, DCMD_SPI_DATA_XCHNG, sparts, rparts, siov, riov, nullptr);
    _cs_release();

    if (r != EOK) {
        // at most once a second, as a failing device fails every transfer
        const uint32_t now_ms = AP_HAL::millis();
        if (_last_error_ms == 0 || now_ms - _last_error_ms >= 1000) {
            _last_error_ms = now_ms;
            hal.console->printf("SPIDevice: error transferring data on %s (%s)\n",
                                _desc.node, strerror(r));
        }
        return false;
    }

    return true;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (!send) {
        send_len = 0;
    }
    if (!recv) {
        recv_len = 0;
    }
    if (send_len == 0 && recv_len == 0) {
        return false;
    }

    /*
      QNX difference: Linux sends a write then a read as two messages of
      one ioctl; io-spi exchanges send and receive the same number of
      bytes, so this is one exchange of send_len + recv_len bytes, the
      chip select held throughout as on Linux, that sends zeros while
      reading and drops what came in while writing.
     */
    if (recv_len == 0) {
        return _exchange(send, send_len, nullptr, send_len, send_len);
    }
    return _exchange(send, send_len, recv, send_len, send_len + recv_len);
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                                    uint32_t len)
{
    if (!send || !recv || len == 0) {
        return false;
    }

    return _exchange(send, len, recv, 0, len);
}

bool SPIDevice::transfer_fullduplex(uint8_t *send_recv, uint32_t len)
{
    return transfer_fullduplex(send_recv, send_recv, len);
}

void SPIDevice::_cs_assert()
{
    if (_desc.cs_pin == SPI_CS_KERNEL) {
        return;
    }

    _cs->write(0);
}

void SPIDevice::_cs_release()
{
    if (_desc.cs_pin == SPI_CS_KERNEL) {
        return;
    }

    _cs->write(1);
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &_bus.semaphore();
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(
    uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _bus.register_periodic_callback(period_usec, cb);
}

bool SPIDevice::adjust_periodic_callback(
    AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return _bus.adjust_periodic_callback(h, period_usec);
}

bool SPIDevice::unregister_callback(AP_HAL::Device::PeriodicHandle h)
{
    return _bus.unregister_callback(h);
}

AP_HAL::SPIDevice *
SPIDeviceManager::get_device_ptr(const char *name)
{
    SPIDesc *desc = nullptr;

    /* Find the bus description in the table */
    for (uint8_t i = 0; i < _n_device_desc; i++) {
        if (!strcmp(_device[i].name, name)) {
            desc = &_device[i];
            break;
        }
    }

    if (!desc) {
        return nullptr;
    }

    if (desc->bus >= ARRAY_SIZE(_buses)) {
        hal.console->printf("SPI: bus %u of %s is out of range\n", unsigned(desc->bus), name);
        return nullptr;
    }

    WITH_SEMAPHORE(_buses_semaphore);

    if (_buses[desc->bus] == nullptr) {
        /* Bus not found for this device, create a new one */
        _buses[desc->bus] = NEW_NOTHROW SPIBus(desc->bus);
        if (_buses[desc->bus] == nullptr) {
            return nullptr;
        }
    }
    SPIBus &b = *_buses[desc->bus];

    int fd;
    {
        WITH_SEMAPHORE(b.semaphore());
        fd = b.open(desc->node);
    }
    if (fd < 0) {
        return nullptr;
    }

    return NEW_NOTHROW SPIDevice(b, *desc, fd);
}

uint8_t SPIDeviceManager::get_count()
{
   return _n_device_desc;
}

const char* SPIDeviceManager::get_device_name(uint8_t idx)
{
    if (idx >= _n_device_desc) {
        return nullptr;
    }
    return _device[idx].name;
}

void SPIDeviceManager::teardown()
{
    for (SPIBus *b : _buses) {
        if (b != nullptr) {
            b->teardown();
        }
    }
}

}
