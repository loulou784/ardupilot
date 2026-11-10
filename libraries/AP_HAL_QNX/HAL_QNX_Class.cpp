
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QNX

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include <AP_HAL/utility/getopt_cpp.h>

#include "HAL_QNX_Class.h"
#include "AP_HAL_QNX_Private.h"

using namespace QNX;

// 10 serial ports, as on Linux
static UARTDriver serial0Driver(true);
static UARTDriver serial1Driver(false);
static UARTDriver serial2Driver(false);
static UARTDriver serial3Driver(false);
static UARTDriver serial4Driver(false);
static UARTDriver serial5Driver(false);
static UARTDriver serial6Driver(false);
static UARTDriver serial7Driver(false);
static UARTDriver serial8Driver(false);
static UARTDriver serial9Driver(false);

static UARTDriver* serialDrivers[] = {
    &serial0Driver,
    &serial1Driver,
    &serial2Driver,
    &serial3Driver,
    &serial4Driver,
    &serial5Driver,
    &serial6Driver,
    &serial7Driver,
    &serial8Driver,
    &serial9Driver,
};

static I2CDeviceManager i2cDeviceManager;
static SPIDeviceManager spiDeviceManager;
static WSPIDeviceManager wspiDeviceManager;
static AnalogIn analogIn;
static Storage storageDriver;
static GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;
static Scheduler schedulerInstance;
static Util utilInstance;
static OpticalFlow opticalFlowDriver;
static Flash flashDriver;
#if HAL_WITH_DSP
static DSP dspDriver;
#endif

HAL_QNX::HAL_QNX() :
    AP_HAL::HAL(
        &serial0Driver,
        &serial1Driver,
        &serial2Driver,
        &serial3Driver,
        &serial4Driver,
        &serial5Driver,
        &serial6Driver,
        &serial7Driver,
        &serial8Driver,
        &serial9Driver,
        &i2cDeviceManager,
        &spiDeviceManager,
        &wspiDeviceManager,
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
#if HAL_WITH_DSP
        &dspDriver,
#endif
        nullptr)            /* no CAN */
{}

void _usage(void)
{
    printf("Usage: --serial0 serial0Path --serial1 serial1Path \n");
    printf("Examples:\n");
    printf("\tserial (0 through 9 available):\n");
    printf("\t                  --serial0 /dev/ser1\n");
    printf("\t                  --serial3 /dev/ser2\n");
    printf("\tlegacy UART options are deprecated, their mappings are:\n");
    printf("\t                  -A/--uartA is SERIAL0\n");
    printf("\t                  -C/--uartC is SERIAL1\n"); // ordering captures the historical use of uartB as SERIAL3
    printf("\t                  -D/--uartD is SERIAL2\n");
    printf("\t                  -B/--uartB is SERIAL3\n");
    printf("\t                  -E/--uartE is SERIAL4\n");
    printf("\t                  -F/--uartF is SERIAL5\n");
    printf("\t                  -G/--uartG is SERIAL6\n");
    printf("\t                  -H/--uartH is SERIAL7\n");
    printf("\t                  -I/--uartI is SERIAL8\n");
    printf("\t                  -J/--uartJ is SERIAL9\n");
    printf("\tnetworking tcp:\n");
    printf("\t                  --serial1 tcp:192.168.2.15:1243:wait\n");
    printf("\t                  --serial0 tcp:11.0.0.2:5678\n");
    printf("\t                  --serial0 udp:11.0.0.2:14550\n");
    printf("\tnetworking UDP:\n");
    printf("\t                  --serial0 udp:11.0.0.255:14550:bcast\n");
    printf("\t                  --serial0 udpin:0.0.0.0:14550\n");
}

void HAL_QNX::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    enum long_options {
        CMDLINE_SERIAL0 = 1, // must be in 0-9 order and numbered consecutively
        CMDLINE_SERIAL1,
        CMDLINE_SERIAL2,
        CMDLINE_SERIAL3,
        CMDLINE_SERIAL4,
        CMDLINE_SERIAL5,
        CMDLINE_SERIAL6,
        CMDLINE_SERIAL7,
        CMDLINE_SERIAL8,
        CMDLINE_SERIAL9,
    };

    /*
      the options of AP_HAL_Linux that set serial ports.  Its directory,
      defaults and CPU affinity options set values in Util, and come with
      the Util port.
     */
    int opt;
    const struct GetOptLong::option options[] = {
        {"uartA",         true,  0, 'A'},
        {"uartB",         true,  0, 'B'},
        {"uartC",         true,  0, 'C'},
        {"uartD",         true,  0, 'D'},
        {"uartE",         true,  0, 'E'},
        {"uartF",         true,  0, 'F'},
        {"uartG",         true,  0, 'G'},
        {"uartH",         true,  0, 'H'},
        {"uartI",         true,  0, 'I'},
        {"uartJ",         true,  0, 'J'},
        {"serial0",       true,  0, CMDLINE_SERIAL0},
        {"serial1",       true,  0, CMDLINE_SERIAL1},
        {"serial2",       true,  0, CMDLINE_SERIAL2},
        {"serial3",       true,  0, CMDLINE_SERIAL3},
        {"serial4",       true,  0, CMDLINE_SERIAL4},
        {"serial5",       true,  0, CMDLINE_SERIAL5},
        {"serial6",       true,  0, CMDLINE_SERIAL6},
        {"serial7",       true,  0, CMDLINE_SERIAL7},
        {"serial8",       true,  0, CMDLINE_SERIAL8},
        {"serial9",       true,  0, CMDLINE_SERIAL9},
        {"help",                false,  0, 'h'},
        {0, false, 0, 0}
    };

    GetOptLong gopt(argc, argv, "A:B:C:D:E:F:G:H:I:J:h",
                    options);

    /*
      parse command line options
     */
    while ((opt = gopt.getoption()) != -1) {
        switch (opt) {
        case 'A':
        case 'B':
        case 'C':
        case 'D':
        case 'E':
        case 'F':
        case 'G':
        case 'H':
        case 'I':
        case 'J': {
            int uart_idx = opt - 'A';
            // ordering captures the historical use of uartB as SERIAL3
            static const uint8_t mapping[] = { 0, 3, 1, 2, 4, 5, 6, 7, 8, 9 };
            int serial_idx = mapping[uart_idx];
            printf("WARNING: deprecated option --uart%c/-%c will be removed in "
                "a future release. Use --serial%d instead.\n",
                (char)opt, (char)opt, serial_idx);
            serialDrivers[serial_idx]->set_device_path(gopt.optarg);
            break;
        }
        case CMDLINE_SERIAL0:
        case CMDLINE_SERIAL1:
        case CMDLINE_SERIAL2:
        case CMDLINE_SERIAL3:
        case CMDLINE_SERIAL4:
        case CMDLINE_SERIAL5:
        case CMDLINE_SERIAL6:
        case CMDLINE_SERIAL7:
        case CMDLINE_SERIAL8:
        case CMDLINE_SERIAL9:
            serialDrivers[opt - CMDLINE_SERIAL0]->set_device_path(gopt.optarg);
            break;
        case 'h':
            _usage();
            exit(0);
        default:
            printf("Unknown option '%c'\n", (char)opt);
            exit(1);
        }
    }

    // NOTE: signal handlers are only set before the main loop, so
    // that if anything before the loops hangs, the default signals
    // can still stop the process proprely, although without proper
    // teardown.
    // This isn't perfect, but still prevents an unkillable process.

    scheduler->init();
    gpio->init();
    rcout->init();
    rcin->init();
    serial(0)->begin(115200);
    analogin->init();

    // As on Linux, release the scheduler threads before setup(), so
    // that they serve the drivers (UARTs, storage, IO processes) while
    // the vehicle starts.  See commit 9f5b4ffca ("AP_HAL_Linux_Class:
    // Correct deadlock, and infinite loop in setup()").
    scheduler->set_system_initialized();

    callbacks->setup();

    setup_signal_handlers();

    while (!_should_exit) {
        callbacks->loop();
    }

    // At least try to stop all PWM before shutting down
    rcout->force_safety_on();
    rcin->teardown();
    Scheduler::from(scheduler)->teardown();
}

void HAL_QNX::setup_signal_handlers() const
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = HAL_QNX::exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
}

static HAL_QNX hal_qnx;

void HAL_QNX::exit_signal_handler(int signum)
{
    hal_qnx._should_exit = true;
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    return hal_qnx;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable() {
    return hal_qnx;
}

#endif
