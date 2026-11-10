#pragma once

#include <AP_HAL/AP_HAL.h>

#include <signal.h>

#include "AP_HAL_QNX_Namespace.h"

class HAL_QNX : public AP_HAL::HAL {
public:
    HAL_QNX();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;

    void setup_signal_handlers() const;

    static void exit_signal_handler(int);

protected:
    volatile sig_atomic_t _should_exit = false;
};
