#include "RCInput.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_RCProtocol/AP_RCProtocol.h>

using namespace QNX;

RCInput::RCInput()
{}

/*
  RC input comes from AP_RCProtocol, as on ChibiOS: a receiver on a
  serial port set to RCIN (SERIALn_PROTOCOL 23), eg. CRSF, SBUS through an
  inverter, DSM or SRXL.  IcePilot's RC input pin, through the FPGA, is to
  come (3.8).  With simulation on hardware, AP_RCProtocol's UDP backend
  simulates a receiver with the sticks centred and the throttle low, as
  in AP_HAL_SITL.

  With no receiver, there are no channels, so the vehicle sees no RC and
  its RC failsafe works; the previous stub reported centred sticks and a
  low throttle, as if a receiver were there.
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

const char *RCInput::protocol() const
{
    return AP::RC().detected_protocol_name();
}
