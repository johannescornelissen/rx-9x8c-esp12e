#include "sbus.h"

SBUSClass::SBUSClass()
{
    fSerial = 0;
}

void SBUSClass::begin(HardwareSerial *serial)
{
    fSerial = serial;
    if (fSerial)
        fSerial->begin(100000, SERIAL_8E2, SERIAL_TX_ONLY);
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
        return fSerial->write((uint8_t *)sbus_frame, sizeof(*sbus_frame)) == sizeof(*sbus_frame);
    else
        return false;
}

#define CHANNEL_MAPPING(chan_) channel_count >= chan_ ? channels[chan_ - 1] : SBUS_UNDEFINED_CHANNEL_VALUE
#define CHANNEL_MAPPING_BOOL(chan_) channel_count >= chan_ ? channels[chan_ - 1] >= 1500 : false

sbus_frame_t *SBUSClass::build_sbus_frame(
    sbus_frame_t *sbus_frame,
    int *channels, int channel_count,
    bool frame_lost, bool fail_safe_activated, int counter)
{
    // header
    sbus_frame->startbyte = SBUS_START_BYTE;
    // normal servo channels
    sbus_frame->channels.ch1 = CHANNEL_MAPPING(1);
    sbus_frame->channels.ch2 = CHANNEL_MAPPING(2);
    sbus_frame->channels.ch3 = CHANNEL_MAPPING(3);
    sbus_frame->channels.ch4 = CHANNEL_MAPPING(4);
    sbus_frame->channels.ch5 = CHANNEL_MAPPING(5);
    sbus_frame->channels.ch6 = CHANNEL_MAPPING(6);
    sbus_frame->channels.ch7 = CHANNEL_MAPPING(7);
    sbus_frame->channels.ch8 = CHANNEL_MAPPING(8);
    sbus_frame->channels.ch9 = CHANNEL_MAPPING(9);
    sbus_frame->channels.ch10 = CHANNEL_MAPPING(10);
    sbus_frame->channels.ch11 = CHANNEL_MAPPING(11);
    sbus_frame->channels.ch12 = CHANNEL_MAPPING(12);
    sbus_frame->channels.ch13 = CHANNEL_MAPPING(13);
    sbus_frame->channels.ch14 = CHANNEL_MAPPING(14);
    sbus_frame->channels.ch15 = CHANNEL_MAPPING(15);
    sbus_frame->channels.ch16 = CHANNEL_MAPPING(16);
    // special on/off servo channels
    sbus_frame->channels.ch17 = CHANNEL_MAPPING_BOOL(17);
    sbus_frame->channels.ch18 = CHANNEL_MAPPING_BOOL(18);
    // status
    sbus_frame->channels.Frame_lost = frame_lost;
    sbus_frame->channels.failsafe_activated = fail_safe_activated;
    sbus_frame->channels.counter = counter;
    // footer
    sbus_frame->endbyte = SBUS_END_BYTE;
    return sbus_frame;
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SBUS)
SBUSClass SBUS;
#endif