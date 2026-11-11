#include <AP_HAL/AP_HAL.h>

#include "GPIO.h"

#if HAL_QNX_GPIO_RPI_ENABLED

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/iomgr.h>
#include <sys/iomsg.h>
#include <sys/neutrino.h>
#include <unistd.h>

using namespace QNX;

extern const AP_HAL::HAL& hal;

namespace {

/*
  The rpi_gpio messages, from rpi_gpio.h in the QNX hardware component
  samples (gitlab.com/qnx/projects/hardware-component-samples), which are
  not part of the SDP.
 */
constexpr const char *RPI_GPIO_PATH = "/dev/gpio/msg";
constexpr uint16_t RPI_GPIO_IOMGR = _IOMGR_PRIVATE_BASE + 35;

enum : uint16_t {
    RPI_GPIO_SET_SELECT,
    RPI_GPIO_GET_SELECT,
    RPI_GPIO_WRITE,
    RPI_GPIO_READ,
    RPI_GPIO_ADD_EVENT,
};

enum : uint32_t {
    RPI_EVENT_NONE = 0,
    RPI_EVENT_EDGE_RISING = 0x1,
    RPI_EVENT_EDGE_FALLING = 0x2,
};

// pin functions, as the BCM2711 encodes them
enum : uint32_t {
    RPI_GPIO_FUNC_IN = 0,
    RPI_GPIO_FUNC_OUT = 1,
};
const uint32_t rpi_gpio_func_alt[] = { 4, 5, 6, 7, 3, 2 };    // ALT0 to ALT5

struct rpi_gpio_msg_t {
    struct _io_msg hdr;
    unsigned gpio;
    unsigned value;
};

struct rpi_gpio_event_t {
    struct _io_msg hdr;
    unsigned gpio;
    unsigned detect;
    struct sigevent event;
    unsigned match;
    unsigned reserved;
};

// pin interrupts are handled at the priority of the timer thread
constexpr int APM_QNX_GPIO_PRIORITY = 15;
constexpr int PULSE_CODE_GPIO = _PULSE_CODE_MINAVAIL;

// errors after which the connection to rpi_gpio is opened again
bool connection_lost(int err)
{
    return err == EBADF || err == ESRCH || err == ENOTCONN || err == ECONNRESET;
}

}

GPIO_RPI::GPIO_RPI()
{
}

void GPIO_RPI::init()
{
    WITH_SEMAPHORE(_connect_semaphore);
    _fd = ::open(RPI_GPIO_PATH, O_RDWR | O_CLOEXEC);
    if (_fd < 0) {
        // GPIO is optional for flying: say why it does not work, and
        // try again later
        hal.console->printf("GPIO: unable to open %s: %s\n", RPI_GPIO_PATH, strerror(errno));
    }
    _last_connect_ms = AP_HAL::millis();
}

bool GPIO_RPI::valid_pin(uint8_t pin) const
{
    return pin < NUM_PINS && (HAL_QNX_GPIO_PIN_MASK & (1UL << pin)) != 0;
}

void GPIO_RPI::_warn(const char *what, uint8_t pin, int err)
{
    // once per pin, as a failing pin fails on every use
    const uint32_t bit = 1UL << (pin < 32 ? pin : 31);
    if (_warned_mask & bit) {
        return;
    }
    _warned_mask |= bit;
    hal.console->printf("GPIO: %s pin %u failed: %s\n", what, unsigned(pin), strerror(err));
}

bool GPIO_RPI::_reconnect()
{
    WITH_SEMAPHORE(_connect_semaphore);
    // at most once a second, as rpi_gpio may be gone for good
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_connect_ms < 1000) {
        return false;
    }
    _last_connect_ms = now_ms;
    if (_fd >= 0) {
        ::close(_fd);
    }
    _fd = ::open(RPI_GPIO_PATH, O_RDWR | O_CLOEXEC);
    if (_fd < 0) {
        return false;
    }
    hal.console->printf("GPIO: reconnected to %s\n", RPI_GPIO_PATH);

    // the pin interrupts belonged to the old connection
    WITH_SEMAPHORE(_interrupts_semaphore);
    for (uint8_t pin = 0; pin < NUM_PINS; pin++) {
        Interrupt &irq = _interrupts[pin];
        if (irq.mode != INTERRUPT_NONE) {
            _attach(pin, irq.fn, irq.proc, irq.mode);
        }
    }
    return true;
}

bool GPIO_RPI::_message(uint16_t subtype, uint8_t pin, uint32_t value, uint32_t *reply)
{
    if (!valid_pin(pin)) {
        return false;
    }
    rpi_gpio_msg_t msg {};
    msg.hdr.type = _IO_MSG;
    msg.hdr.subtype = subtype;
    msg.hdr.mgrid = RPI_GPIO_IOMGR;
    msg.gpio = pin;
    msg.value = value;

    for (uint8_t attempt = 0; attempt < 2; attempt++) {
        if (_fd < 0 && !_reconnect()) {
            return false;
        }
        // threads may send on the same connection at the same time
        if (MsgSend(_fd, &msg, sizeof(msg), &msg, reply ? sizeof(msg) : 0) != -1) {
            if (reply) {
                *reply = msg.value;
            }
            return true;
        }
        const int err = errno;
        if (!connection_lost(err) || !_reconnect()) {
            _warn(subtype == RPI_GPIO_READ ? "reading" : "setting", pin, err);
            return false;
        }
    }
    return false;
}

bool GPIO_RPI::_set_function(uint8_t pin, uint32_t function)
{
    if (!_message(RPI_GPIO_SET_SELECT, pin, function)) {
        return false;
    }
    const uint32_t bit = 1UL << pin;
    if (function == RPI_GPIO_FUNC_OUT) {
        __atomic_or_fetch(&_output_mask, bit, __ATOMIC_RELAXED);
    } else {
        __atomic_and_fetch(&_output_mask, ~bit, __ATOMIC_RELAXED);
    }
    return true;
}

void GPIO_RPI::pinMode(uint8_t pin, uint8_t output)
{
    if (output == HAL_GPIO_ALT) {
        // ALT with no function given: ALT0, as on Linux
        pinMode(pin, output, 0);
        return;
    }
    _set_function(pin, output ? RPI_GPIO_FUNC_OUT : RPI_GPIO_FUNC_IN);
}

void GPIO_RPI::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    if (output != HAL_GPIO_ALT) {
        pinMode(pin, output);
        return;
    }
    if (alt >= ARRAY_SIZE(rpi_gpio_func_alt)) {
        return;
    }
    _set_function(pin, rpi_gpio_func_alt[alt]);
}

uint8_t GPIO_RPI::read(uint8_t pin)
{
    if (!valid_pin(pin)) {
        return 0;
    }
    /*
      rpi_gpio reads only inputs, and fails with ENXIO on an output: an
      output reads as the level last written to it, as reading the
      output latch does on other boards
     */
    const uint32_t bit = 1UL << pin;
    if (__atomic_load_n(&_output_mask, __ATOMIC_RELAXED) & bit) {
        return (__atomic_load_n(&_level_mask, __ATOMIC_RELAXED) & bit) ? 1 : 0;
    }
    uint32_t value = 0;
    if (!_message(RPI_GPIO_READ, pin, 0, &value)) {
        return 0;
    }
    return value ? 1 : 0;
}

void GPIO_RPI::write(uint8_t pin, uint8_t value)
{
    if (!_message(RPI_GPIO_WRITE, pin, value ? 1 : 0)) {
        return;
    }
    const uint32_t bit = 1UL << pin;
    if (value) {
        __atomic_or_fetch(&_level_mask, bit, __ATOMIC_RELAXED);
    } else {
        __atomic_and_fetch(&_level_mask, ~bit, __ATOMIC_RELAXED);
    }
}

void GPIO_RPI::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_RPI::channel(uint16_t n)
{
    if (n > UINT8_MAX || !valid_pin(n)) {
        return nullptr;
    }
    return NEW_NOTHROW DigitalSource(n);
}

bool GPIO_RPI::attach_interrupt(uint8_t pin, irq_handler_fn_t fn,
                                INTERRUPT_TRIGGER_TYPE mode)
{
    if (!fn) {
        mode = INTERRUPT_NONE;
    }
    WITH_SEMAPHORE(_interrupts_semaphore);
    return _attach(pin, fn, nullptr, mode);
}

bool GPIO_RPI::attach_interrupt(uint8_t pin, AP_HAL::Proc proc,
                                INTERRUPT_TRIGGER_TYPE mode)
{
    if (proc == nullptr) {
        mode = INTERRUPT_NONE;
    }
    WITH_SEMAPHORE(_interrupts_semaphore);
    return _attach(pin, nullptr, proc, mode);
}

bool GPIO_RPI::_attach(uint8_t pin, irq_handler_fn_t fn, AP_HAL::Proc proc,
                       INTERRUPT_TRIGGER_TYPE mode)
{
    if (!valid_pin(pin)) {
        return false;
    }
    Interrupt &irq = _interrupts[pin];
    if (mode != INTERRUPT_NONE && irq.mode != INTERRUPT_NONE &&
        (irq.fn != fn || irq.proc != proc)) {
        // as on ChibiOS: one handler per pin
        return false;
    }
    if (mode != INTERRUPT_NONE && !_start_interrupt_thread()) {
        return false;
    }

    rpi_gpio_event_t ev {};
    ev.hdr.type = _IO_MSG;
    ev.hdr.subtype = RPI_GPIO_ADD_EVENT;
    ev.hdr.mgrid = RPI_GPIO_IOMGR;
    ev.gpio = pin;
    switch (mode) {
    case INTERRUPT_NONE:
        // rpi_gpio has one event per pin, and none detects nothing
        ev.detect = RPI_EVENT_NONE;
        break;
    case INTERRUPT_RISING:
        ev.detect = RPI_EVENT_EDGE_RISING;
        break;
    case INTERRUPT_FALLING:
        ev.detect = RPI_EVENT_EDGE_FALLING;
        break;
    case INTERRUPT_BOTH:
        ev.detect = RPI_EVENT_EDGE_RISING | RPI_EVENT_EDGE_FALLING;
        break;
    }

    if (_fd < 0) {
        if (mode == INTERRUPT_NONE) {
            // no connection, so no event either
            irq = Interrupt {};
            return true;
        }
        return false;
    }
    if (ev.detect != RPI_EVENT_NONE) {
        SIGEV_PULSE_INIT(&ev.event, _coid, APM_QNX_GPIO_PRIORITY, PULSE_CODE_GPIO, pin);
        // QNX 8 delivers only events registered with the server
        if (MsgRegisterEvent(&ev.event, _fd) == -1) {
            _warn("registering an interrupt on", pin, errno);
            return false;
        }
    }
    if (MsgSend(_fd, &ev, sizeof(ev), nullptr, 0) == -1) {
        _warn("setting an interrupt on", pin, errno);
        return false;
    }

    if (mode == INTERRUPT_NONE) {
        irq = Interrupt {};
    } else {
        irq.fn = fn;
        irq.proc = proc;
        irq.mode = mode;
    }
    return true;
}

bool GPIO_RPI::_start_interrupt_thread()
{
    if (_thread.is_started()) {
        return true;
    }
    // with a fixed priority, as each pulse would otherwise set the
    // thread's priority to the pulse's
    _chid = ChannelCreate(_NTO_CHF_PRIVATE | _NTO_CHF_FIXED_PRIORITY);
    if (_chid == -1) {
        return false;
    }
    _coid = ConnectAttach(0, 0, _chid, _NTO_SIDE_CHANNEL, 0);
    if (_coid == -1) {
        ChannelDestroy(_chid);
        _chid = -1;
        return false;
    }
    _thread.set_stack_size(64 * 1024);
    if (!_thread.start("ap-gpio", SCHED_FIFO, APM_QNX_GPIO_PRIORITY)) {
        ConnectDetach(_coid);
        ChannelDestroy(_chid);
        _coid = _chid = -1;
        return false;
    }
    return true;
}

void GPIO_RPI::_interrupt_thread()
{
    while (true) {
        struct _pulse pulse;
        if (MsgReceivePulse(_chid, &pulse, sizeof(pulse), nullptr) == -1) {
            if (errno == EINTR) {
                continue;
            }
            hal.console->printf("GPIO: interrupt thread stopped: %s\n", strerror(errno));
            return;
        }
        if (pulse.code != PULSE_CODE_GPIO) {
            continue;
        }
        // the pulse carries no level or time: time it on arrival
        const uint32_t now_us = AP_HAL::micros();
        const int pin = pulse.value.sival_int;
        if (pin < 0 || pin >= NUM_PINS) {
            continue;
        }
        Interrupt irq;
        {
            WITH_SEMAPHORE(_interrupts_semaphore);
            irq = _interrupts[pin];
        }
        bool state;
        switch (irq.mode) {
        case INTERRUPT_RISING:
            state = true;
            break;
        case INTERRUPT_FALLING:
            state = false;
            break;
        case INTERRUPT_BOTH:
            state = read(pin) != 0;
            break;
        case INTERRUPT_NONE:
        default:
            // a pulse sent before the interrupt was detached
            continue;
        }
        if (irq.fn) {
            irq.fn(pin, state, now_us);
        } else if (irq.proc) {
            irq.proc();
        }
    }
}

bool GPIO_RPI::usb_connected(void)
{
    return false;
}

#endif  // HAL_QNX_GPIO_RPI_ENABLED
