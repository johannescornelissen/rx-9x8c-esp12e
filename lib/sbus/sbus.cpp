#include "sbus.h"

SBUSClass::SBUSClass()
{
    fSerial = 0;
}

void SBUSClass::begin(HardwareSerial *serial)
{
    fSerial = serial;
    if (fSerial)
        fSerial->begin(100000, SERIAL_8E2);
}
void SBUSClass::end()
{
    if (fSerial)
    {
        fSerial->end();
        fSerial = 0;
    }
}

bool SBUSClass::send_frame(sbus_frame_t *sbus_frame)
{
    if (fSerial)
        return fSerial->write((uint8_t *)sbus_frame, sizeof(*sbus_frame))==sizeof(*sbus_frame);
    else
        return false;
}

sbus_frame_t *SBUSClass::build_sbus_frame(
    sbus_frame_t *sbus_frame, 
    int *channels, int channel_count, bool ch17, bool ch18, 
    bool frame_lost, bool fail_safe_activated, int counter)
{
    sbus_frame->startbyte = SBUS_START_BYTE;
    sbus_frame->ch1 = channel_count >= 1 ? channels[1 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch2 = channel_count >= 2 ? channels[2 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch3 = channel_count >= 3 ? channels[3 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch4 = channel_count >= 4 ? channels[4 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch5 = channel_count >= 5 ? channels[5 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch6 = channel_count >= 6 ? channels[6 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch7 = channel_count >= 7 ? channels[7 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch8 = channel_count >= 8 ? channels[8 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch9 = channel_count >= 9 ? channels[9 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch10 = channel_count >= 10 ? channels[10 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch11 = channel_count >= 11 ? channels[11 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch12 = channel_count >= 12 ? channels[12 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch13 = channel_count >= 13 ? channels[13 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch14 = channel_count >= 14 ? channels[14 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch15 = channel_count >= 15 ? channels[15 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->ch16 = channel_count >= 16 ? channels[16 - 1] : SBUS_UNDEFINED_CHANNEL_VALUE;
    sbus_frame->flags.ch17 = ch17;
    sbus_frame->flags.ch18 = ch18;
    sbus_frame->flags.Frame_lost = frame_lost;
    sbus_frame->flags.failsafe_activated = fail_safe_activated;
    sbus_frame->flags.counter = counter;
    sbus_frame->endbyte = SBUS_END_BYTE;
    return sbus_frame;
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SBUS)
SBUSClass SBUS;
#endif