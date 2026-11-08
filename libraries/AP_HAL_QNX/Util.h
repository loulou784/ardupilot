#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_QNX_Namespace.h"

class QNX::Util : public AP_HAL::Util {
public:
    uint64_t get_hw_rtc() const override { return 0; }
    void set_hw_rtc(uint64_t time_utc_usec) override {}
};
