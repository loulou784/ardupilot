#pragma once

#include <stdint.h>

#include "AP_HAL_QNX.h"
#include "Semaphores.h"
#include "Thread.h"

// the pins of the 40-pin header, GPIO0 to GPIO27, that ArduPilot may
// use; a board leaves out the pins its buses and serial ports use, so
// that a parameter cannot take them over
#ifndef HAL_QNX_GPIO_PIN_MASK
#define HAL_QNX_GPIO_PIN_MASK 0x0FFFFFFFU
#endif

namespace QNX {

/*
  GPIO on the Raspberry Pi, through the rpi_gpio resource manager of the
  QNX Raspberry Pi images, at /dev/gpio/msg.

  QNX difference: Linux's GPIO_RPI maps the GPIO registers through
  /dev/gpiomem; QNX has no /dev/gpiomem, and mapping the registers needs
  root, so each operation is a message to rpi_gpio, which takes about
  5 us on the Raspberry Pi 4.  Unlike Linux, pin interrupts are
  supported: rpi_gpio sends a pulse on each edge, which a thread turns
  into the handler call, 10 to 20 us after the edge.
 */
class GPIO_RPI : public AP_HAL::GPIO {
public:
    GPIO_RPI();
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;
    bool    valid_pin(uint8_t pin) const override;

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /* Interrupt interface: */
    bool attach_interrupt(uint8_t pin,
                          irq_handler_fn_t fn,
                          INTERRUPT_TRIGGER_TYPE mode) override;
    bool attach_interrupt(uint8_t pin,
                          AP_HAL::Proc proc,
                          INTERRUPT_TRIGGER_TYPE mode) override;

    /* return true if USB cable is connected */
    bool    usb_connected(void) override;

    static constexpr uint8_t NUM_PINS = 28;

private:
    // one message to rpi_gpio; false if it failed
    bool _message(uint16_t subtype, uint8_t pin, uint32_t value, uint32_t *reply = nullptr);
    bool _reconnect();
    bool _set_function(uint8_t pin, uint32_t function);
    bool _attach(uint8_t pin, irq_handler_fn_t fn, AP_HAL::Proc proc,
                 INTERRUPT_TRIGGER_TYPE mode);
    bool _start_interrupt_thread();
    void _interrupt_thread();
    void _warn(const char *what, uint8_t pin, int err);

    int _fd = -1;
    Semaphore _connect_semaphore;
    uint32_t _last_connect_ms = 0;

    struct Interrupt {
        irq_handler_fn_t fn;
        AP_HAL::Proc proc = nullptr;
        INTERRUPT_TRIGGER_TYPE mode = INTERRUPT_NONE;
    };
    Interrupt _interrupts[NUM_PINS] {};
    Semaphore _interrupts_semaphore;
    int _chid = -1;
    int _coid = -1;
    Thread _thread{FUNCTOR_BIND_MEMBER(&GPIO_RPI::_interrupt_thread, void)};

    uint32_t _warned_mask = 0;

    // the pins set as outputs, and the levels last written to them
    uint32_t _output_mask = 0;
    uint32_t _level_mask = 0;
};

}
