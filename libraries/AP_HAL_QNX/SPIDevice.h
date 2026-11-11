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
#pragma once

#include <inttypes.h>

#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>

#include "Semaphores.h"

#ifndef HAL_QNX_SPI_MAX_BUSES
#define HAL_QNX_SPI_MAX_BUSES 8
#endif

namespace QNX {

class SPIBus;

struct SPIDesc {
    /*
      QNX difference: QNX names each SPI device node in the io-spi
      configuration, so the board gives the node, where Linux opens
      /dev/spidev<bus>.<subdev>.  bus and subdev are the device's numbers
      in the device ID, as on Linux.
     */
    SPIDesc(const char *name_, uint16_t bus_, uint16_t subdev_, const char *node_,
            uint32_t mode_, uint8_t bits_per_word_, int16_t cs_pin_,
            uint32_t lowspeed_, uint32_t highspeed_)
        : name(name_), bus(bus_), subdev(subdev_), node(node_), mode(mode_)
        , bits_per_word(bits_per_word_), cs_pin(cs_pin_), lowspeed(lowspeed_)
        , highspeed(highspeed_)
    {
    }

    const char *name;
    uint16_t bus;
    uint16_t subdev;
    const char *node;
    uint32_t mode;
    uint8_t bits_per_word;
    int16_t cs_pin;
    uint32_t lowspeed;
    uint32_t highspeed;
};

class SPIDevice : public AP_HAL::SPIDevice {
public:
    SPIDevice(SPIBus &bus, SPIDesc &device_desc, int fd);

    virtual ~SPIDevice();

    /* AP_HAL::SPIDevice implementation */

    /* See AP_HAL::Device::set_speed() */
    bool set_speed(AP_HAL::Device::Speed speed) override;

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;

    /* See AP_HAL::SPIDevice::transfer_fullduplex() */
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                             uint32_t len) override;

    /* See AP_HAL::SPIDevice::transfer_fullduplex() */
    bool transfer_fullduplex(uint8_t *send_recv, uint32_t len) override;

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore() override;

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb) override;

    /* See AP_HAL::Device::adjust_periodic_callback() */
    bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override;

    /* See AP_HAL::Device::unregister_callback() */
    bool unregister_callback(AP_HAL::Device::PeriodicHandle h) override;

protected:
    SPIBus &_bus;
    SPIDesc &_desc;
    const int _fd;
    AP_HAL::DigitalSource *_cs = nullptr;
    uint32_t _speed;
    uint32_t _last_error_ms = 0;

    // one exchange of len bytes, with CS held: send_len bytes from send,
    // then zeros; the first skip bytes received are dropped, and the rest
    // stored in recv
    bool _exchange(const uint8_t *send, uint32_t send_len,
                   uint8_t *recv, uint32_t skip, uint32_t len);

    /*
     * Select device if using userspace CS
     */
    void _cs_assert();

    /*
     * Deselect device if using userspace CS
     */
    void _cs_release();
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    friend class SPIDevice;

    static SPIDeviceManager *from(AP_HAL::SPIDeviceManager *spi_mgr)
    {
        return static_cast<SPIDeviceManager*>(spi_mgr);
    }

    SPIDeviceManager() { }

    /* AP_HAL::SPIDeviceManager implementation */
    AP_HAL::SPIDevice *get_device_ptr(const char *name) override;

    /*
     * Stop all SPI threads and block until they are finalized. This doesn't
     * free memory because they can still be used by devices, however device
     * drivers won't receive any new event
     */
    void teardown();

    /* See AP_HAL::SPIDeviceManager::get_count() */
    uint8_t get_count() override;

    /* See AP_HAL::SPIDeviceManager::get_device_name() */
    const char *get_device_name(uint8_t idx) override;

protected:
    /*
      QNX difference: as for I2C, a bus is opened on first use and stays
      open, where Linux closes it with its last device.
     */
    SPIBus *_buses[HAL_QNX_SPI_MAX_BUSES] {};
    Semaphore _buses_semaphore;

    static const uint8_t _n_device_desc;
    static SPIDesc _device[];
};

}
