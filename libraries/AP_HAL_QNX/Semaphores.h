#pragma once

#include "AP_HAL_QNX.h"

class QNX::Semaphore : public AP_HAL::Semaphore {
public:

    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;
    bool check_owner() { return true; }
private:
    bool _taken;
};
