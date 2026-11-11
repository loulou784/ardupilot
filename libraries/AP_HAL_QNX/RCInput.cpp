
#include "RCInput.h"

#include <AP_HAL/AP_HAL.h>
#if AP_SIM_ENABLED
#include <AP_Math/AP_Math.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#endif

using namespace QNX;
RCInput::RCInput()
{}

#if AP_SIM_ENABLED
/*
  With simulation on hardware, RC input comes from AP_RCProtocol, as in
  AP_HAL_SITL: its UDP backend simulates a receiver with the sticks
  centred and the throttle low, and also takes RC input over UDP, as SITL
  does.
 */
void RCInput::init()
{
    AP::RC().init();
}

bool RCInput::new_input()
{
    return AP::RC().new_input();
}

uint8_t RCInput::num_channels()
{
    return AP::RC().num_channels();
}

uint16_t RCInput::read(uint8_t ch)
{
    return AP::RC().read(ch);
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    AP::RC().read(periods, len);
    return MIN(len, num_channels());
}

#else

void RCInput::init()
{}

bool RCInput::new_input() {
    return false;
}

uint8_t RCInput::num_channels() {
    return 0;
}

uint16_t RCInput::read(uint8_t chan) {
    if (chan == 2) return 900; /* throttle should be low, for safety */
    else return 1500;
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len) {
    for (uint8_t i = 0; i < len; i++){
        if (i == 2) periods[i] = 900;
        else periods[i] = 1500;
    }
    return len;
}

#endif  // AP_SIM_ENABLED
