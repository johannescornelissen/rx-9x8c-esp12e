/*
https://forum.arduino.cc/index.php?topic=99708.60
quoted from post:

S-BUS protocol

The protocol is 25 Byte long and is send every 14ms (analog mode) or 7ms (highspeed mode).
One Byte = 1 startbit + 8 databit + 1 paritybit + 2 stopbit (8E2), baudrate = 100'000 bit/s
The highest bit is send first. The logic is inverted (Level High = 1)

[startbyte] [data1] [data2] .... [data22] [flags][endbyte]

startbyte = 11110000b (0xF0)

data 1-22 = [ch1, 11bit][ch2, 11bit] .... [ch16, 11bit] (ch# = 0 till 2047)
channel 1 uses 8 bits from data1 and 3 bits from data2
channel 2 uses last 5 bits from data2 and 6 bits from data3
etc.

flags = bit7 = ch17 = digital channel (0x80)
bit6 = ch18 = digital channel (0x40)
bit5 = Frame lost, equivalent red LED on receiver (0x20)
bit4 = failsafe activated (0x10)
bit3 = n/a
bit2 = n/a
bit1 = n/a
bit0 = n/a

endbyte = 00000000b
*/

#ifndef _SBUS_H_INCLUDED
#define _SBUS_H_INCLUDED

#include <stdint.h>
#include <HardwareSerial.h>

#define SBUS_START_BYTE 0xF0
#define SBUS_END_BYTE 0xF0
#define SBUS_UNDEFINED_CHANNEL_VALUE 1500

typedef struct
{
    uint8_t ch17 : 1;
    uint8_t ch18 : 1;
    uint8_t Frame_lost : 1;
    uint8_t failsafe_activated : 1;
    uint8_t counter : 4;
} sbus_frame_flags_t;

typedef struct
{
    uint8_t startbyte; 
    int ch1 : 11;
    int ch2 : 11;
    int ch3 : 11;
    int ch4 : 11;
    int ch5 : 11;
    int ch6 : 11;
    int ch7 : 11;
    int ch8 : 11;
    int ch9 : 11;
    int ch10 : 11;
    int ch11 : 11;
    int ch12 : 11;
    int ch13 : 11;
    int ch14 : 11;
    int ch15 : 11;
    int ch16 : 11;
    sbus_frame_flags_t flags;
    uint8_t endbyte; 
} sbus_frame_t;

class SBUSClass
{
  public:
    SBUSClass();
    void begin(HardwareSerial *serial);
    void end();
    bool send_frame(sbus_frame_t *sbus_frame);
    static sbus_frame_t *build_sbus_frame(
        sbus_frame_t *sbus_frame,
        int *channels, int channel_count, bool ch17, bool ch18,
        bool frame_lost, bool fail_safe_activated, int counter);

  private:
    HardwareSerial *fSerial;
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SBUS)
extern SBUSClass SBUS;
#endif

#endif
