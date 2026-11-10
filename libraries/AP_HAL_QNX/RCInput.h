#pragma once

#include "AP_HAL_QNX.h"

class QNX::RCInput : public AP_HAL::RCInput {
public:
    RCInput();

    static RCInput *from(AP_HAL::RCInput *rcinput) {
        return static_cast<RCInput*>(rcinput);
    }

    void init() override;
    bool  new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;
    virtual const char *protocol() const override { return "QNX"; }

    // called by the scheduler's rcin thread, as on Linux; empty until
    // there is an RC input backend
    virtual void _timer_tick() {}
};
