#pragma once

#include "AP_HAL_QNX.h"

#ifndef HAL_QNX_GPIO_RPI_ENABLED
#define HAL_QNX_GPIO_RPI_ENABLED 0
#endif

// a board with no GPIO driver
class QNX::GPIO : public AP_HAL::GPIO {
public:
    GPIO();
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;
    bool    valid_pin(uint8_t pin) const override { return false; }

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /* return true if USB cable is connected */
    bool    usb_connected(void) override;
};

// a pin of hal.gpio, as on Linux
class QNX::DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(uint8_t v);
    void    mode(uint8_t output) override;
    uint8_t read() override;
    void    write(uint8_t value) override;
    void    toggle() override;
private:
    uint8_t _v;
};

#if HAL_QNX_GPIO_RPI_ENABLED
#include "GPIO_RPI.h"
#endif
