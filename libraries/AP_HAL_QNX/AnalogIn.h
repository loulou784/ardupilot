#pragma once

#include "AP_HAL_QNX.h"

class QNX::AnalogSource : public AP_HAL::AnalogSource {
public:
    AnalogSource(float v);
    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override { return voltage_average(); }
private:
    float _v;
};

class QNX::AnalogIn : public AP_HAL::AnalogIn {
public:
    AnalogIn();
    void init() override;
    AP_HAL::AnalogSource* channel(int16_t n) override;
    float board_voltage(void) override;
};
