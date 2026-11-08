
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QNX

#include <assert.h>

#include "HAL_QNX_Class.h"
#include "AP_HAL_QNX_Private.h"

using namespace QNX;

static UARTDriver serial0Driver;
static UARTDriver serial1Driver;
static UARTDriver serial2Driver;
static UARTDriver serial3Driver;
static SPIDeviceManager spiDeviceManager;
static AnalogIn analogIn;
static Storage storageDriver;
static GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;
static Scheduler schedulerInstance;
static Util utilInstance;
static OpticalFlow opticalFlowDriver;
static Flash flashDriver;

HAL_QNX::HAL_QNX() :
    AP_HAL::HAL(
        &serial0Driver,
        &serial1Driver,
        &serial2Driver,
        &serial3Driver,
        nullptr,            /* no SERIAL4 */
        nullptr,            /* no SERIAL5 */
        nullptr,            /* no SERIAL6 */
        nullptr,            /* no SERIAL7 */
        nullptr,            /* no SERIAL8 */
        nullptr,            /* no SERIAL9 */
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &serial0Driver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
        nullptr)            /* no DSP */
{}

void HAL_QNX::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init();
    serial(0)->begin(115200);
    _member->init();

    callbacks->setup();
    scheduler->set_system_initialized();

    for (;;) {
        callbacks->loop();
    }
}

static HAL_QNX hal_qnx;

const AP_HAL::HAL& AP_HAL::get_HAL() {
    return hal_qnx;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable() {
    return hal_qnx;
}

#endif
