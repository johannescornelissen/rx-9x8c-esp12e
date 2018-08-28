// esp-12e code for 9x rx simulation with sbus output
// used a lot of code from midelic on RCgroups.com

#include <EEPROM.h>
#include <A7105-SPI.h> // use own extended spi lib
#include <sbus.h>

// config
//#define DEBUG
#define NUMBER_OF_CHANNELS 8
#define FAIL_SAFE_TIME_MILLIS 1500
int fail_safe_servo_values[NUMBER_OF_CHANNELS] = {1500, 1500, 900, 1500, 1500, 1500, 1500, 1500}; // 9x: throttle 900 rest middel to signal failsafe

#define LED_PIN 2 // GPIO2
#define LED_ON LOW
#define LED_OFF HIGH

#define BIND_PIN 4 // GPIO4
#define GIO_PIN 5  // GPIO5

/*
  SPI GPIO13  MOSI  WHITE   R   YELLOW  A7105 DIO
      GPIO12  MISO  YELLOW  -   YELLOW  A7105 DIO
      GPIO14  CLK   GREEN   -   GREEN   A7105 CK
      GPIO15  EN    BROWN   -   WHITE   A7105 CS

  STA GPIO16        BROWN   -   BROWN   A7105 GIO1

  BND GPIO4         BLUE    R   Vcc/GND

  my 9x id seems to be 00035D9B

  https://www.youtube.com/watch?v=tsTZJim1LbY&index=8&list=WL&t=0s

  latency taranis x9d:  23 ms    updates 111hz  sbus
  latency tgy i6s:      15 ms    updates 130hz  ibus

  tip: ftdi connection to esp locks serial monitor on rts/dtr: ctrl-t+ctrl-r and ctrl-t+ctrl-d fixes that

  rx protocols in betaflight
    https://github.com/betaflight/betaflight/tree/master/src/main/rx


  todo: 
  ibus:
    https://basejunction.wordpress.com/2015/08/23/en-flysky-i6-14-channels-part1/
    130 Hz, Serial 115200 bauds, 8n1, header 2B, 14*channel 2B, footer 2B


*/

// initialisation data for 7105, address 0x00-0x32, ff=skip
static const uint8_t A7105_regs[] = {
    0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50, // 0x00-0x0f
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f, // 0x10-0x1f
    0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00, // 0x20-0x2f
    0x01, 0x0f,                                                                                     // 0x30-0x31
};

// channeling switching lookup matrix
static const uint8_t tx_channels[16][16] = {
    {0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0},
    {0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a},
    {0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82},
    {0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a},
    {0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96},
    {0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28},
    {0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64},
    {0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50},
    {0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64},
    {0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50},
    {0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46},
    {0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82},
    {0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46},
    {0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46},
};

// are we in binding mode
bool binding = false;

// transmitter id, stored in EEPROM
volatile uint8_t txid[4] = {0, 0, 0, 0};
uint8_t txid_offset = 0;

// current channel en lookup helpers
int chanrow = 0;
int chanoffset = 0;

// stats helpers
int last_packets = 0;
uint32_t last_millis = 0;

// shared between normal code and interrupt handlers
volatile int chancol = 0;
volatile int this_read_packets = 0;
volatile int other_read_packets = 0;
volatile int invalid_read_packets = 0;
volatile unsigned long last_packet_read_time = 0;
volatile bool fail_safe_mode = false;
volatile int servo_values[NUMBER_OF_CHANNELS]; // also shared between normal code and interrupt handlers

// output
int sbus_frame_counter = 0;

#define txid32 (txid[0] + (txid[1] << 8) + (txid[2] << 16) + (txid[3] << 24))

void setup()
{
  // initialize non-A7105
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ON);
  // debug console
#if defined(DEBUG)
  Serial.begin(74880); // esp default baud rate
  Serial.println("init");
#else
  SBUS.begin(&Serial); // use serial on GPIO1 for SBUS output
#endif
  // check for binding option
  pinMode(BIND_PIN, INPUT_PULLUP);
  binding = digitalRead(BIND_PIN) == 0;
#if defined(DEBUG)
  if (binding)
    Serial.println("binding mode");
  else
    Serial.println("normal mode");
#endif
  // transmitter id from eeprom
  EEPROM.begin(4);
  for (int i = 0; i < 4; i++)
  {
    txid[i] = EEPROM.read(txid_offset + i);
  }
#if defined(DEBUG)
  Serial.print("stored transmitter id ");
  Serial.println(txid32, HEX);
#endif
  // derive channel lookup helpers
  chanrow = txid32 % 16;
  chanoffset = (txid32 & 0xff) / 16;
  if (chanoffset > 9)
    chanoffset = 9; // from sloped soarer findings, bug in flysky protocol
  // use hw spi
  SPI.begin();
  SPI.setHwCs(true);
  // initialize A7105
  SPI.writeb(0x00, 0x00); // reset
  // write id
  SPI.writeid(0x06, 0x5475c52A);
#if defined(DEBUG)
  // read id back
  uint32_t id = SPI.readid(0x06);
  Serial.print("id ");
  Serial.println(id, HEX);
#endif
  // init all byte registers of a7105
  int i;
  for (i = 0; i <= 0x32; i++)
  {
    if (A7105_regs[i] != 0xff)
      SPI.writeb(i, A7105_regs[i]);
  }
  // a7105 calibration
#if defined(DEBUG)
  Serial.print("start calibraton ");
  int t = 0;
#endif
  SPI.strobe(0xA0);             // standby
  SPI.writeb(0x02, 0x01);       // start calibration
  uint16_t d = SPI.readb(0x02); // read calibration status
  while (d == 1)
  {
    delay(1);
#if defined(DEBUG)
    if (++t % 200 == 0)
      Serial.print(".");
#endif
    d = SPI.readb(0x02);
  }
#if defined(DEBUG)
  if (d == 0)
    Serial.println(" done");
  else
  {
    Serial.print(" ## not a valid calibration response (");
    Serial.print(d);
    Serial.println(")");
  }
#endif
  // continue init after calibration
  SPI.readb(0x22); // should return 06?
  SPI.writeb(0x24, 0x13);
  SPI.writeb(0x25, 0x09);
  // at last: GIO pin mode
  pinMode(GIO_PIN, INPUT_PULLUP);
  // initialize servo values to fail safe values
  for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
    servo_values[i] = fail_safe_servo_values[i];
  // only attach interrupt when not in binding mode, that handles reading of packets manually
  if (!binding)
  {
    attachInterrupt(GIO_PIN, handle_new_packet, FALLING);
    next_chan(); //  trigger reading
  }
#if defined(DEBUG)
  Serial.println("init done");
#endif
}

void set_chan(int channel)
{
  SPI.strobe(0xA0);
  SPI.strobe(0xF0);
  if (channel >= 0)
    SPI.writeb(0x0F, channel);
  SPI.strobe(0xC0);
}

void next_chan()
{
  set_chan(tx_channels[chanrow][chancol] - chanoffset - 1);
  chancol = (chancol + 1) % 16;
}

void binding_mode()
{
#if defined(DEBUG)
  Serial.println("entering binding mode");
#endif
  int counter = 0;
  set_chan(0); // listen on channel 0 for binding
  while (1)
  {
    delay(10);
    // toggle led state on bit 3 of counter (8*10 ms): quick led flicker
    digitalWrite(LED_PIN, (counter++ & 8) == 0);
    if (!digitalRead(GIO_PIN))
    {
      uint8_t d = SPI.readb(0x00);
      // test CRC&CRF bits
      if (bitRead(d, 5) == 0)
      {
        uint8_t packet[21];
        SPI.readdata(0x05, packet, sizeof(packet));
        if (packet[0] == 0xaa)
        {
          // bytes 1-4 of packet are transmitter id (observed: byte 0 is 0x55(normal) or 0xaa(binding))
          for (int i = 0; i < 4; i++)
          {
            txid[i] = packet[i + 1];
            // store transmitter address part to eeprom
            EEPROM.write(txid_offset + i, txid[i]);
          }
          EEPROM.commit();
#if defined(DEBUG)
          Serial.print("found transmiter to bind to ");
          Serial.println(txid32, HEX);
#endif
          // continuous lid led: signal we found a transmitter and are bound
          digitalWrite(LED_PIN, LED_ON);
          // endless loop till reset (binding plug should be removed now)
          while (1)
            delay(10); // to avoid watchdog
        }
      }
      // re-init
      set_chan(0);
    }
  }
}

void handle_new_packet()
{
  // read packet from rx on interrupt on GIO1 going low
  // check packet valid flag
  if (bitRead(SPI.readb(0x00), 5) == 0)
  {
    uint8_t packet[21];
    SPI.readdata(0x05, packet, sizeof(packet));
    // check if packet is for us (registered tx id)
    if ((packet[1] == txid[0]) && (packet[2] == txid[1]) && (packet[3] == txid[2]) && (packet[4] == txid[3]))
    {
      // extract new servo values from packet
      for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
      {
        int v = (packet[5 + (2 * i)] + 256 * packet[6 + (2 * i)]);
        if ((v >= 900) && (v <= 2200))
          servo_values[i] = v;
      }
      last_packet_read_time = millis();
      this_read_packets++;
      next_chan();
      // check to reset fail safe mode
      if (fail_safe_mode)
      {
        fail_safe_mode = false;
        digitalWrite(LED_PIN, LED_ON);
      }
    }
    else
    {
      other_read_packets++;
      set_chan(-1); // re-init but do not set channel
    }
  }
  else
  {
    invalid_read_packets++;
    set_chan(-1); // re-init but do not set channel
  }
}

void output_servo_values()
{
  // copy servo values to local copy while interrupts are disabled
  int local_servo_values[NUMBER_OF_CHANNELS];
  // safe region
  noInterrupts();
  for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
    local_servo_values[i] = servo_values[i];
#if defined(DEBUG)
  int local_this_read_packets = this_read_packets;
  int local_other_read_packets = other_read_packets;
  int local_invalid_read_packets = invalid_read_packets;
#endif
  interrupts();

#if defined(DEBUG)
  // show safe local servo values
  for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
  {
    if (i > 0 && i % 2 == 0)
      Serial.print(" | ");
    if (local_servo_values[i] <= 999)
      Serial.print("  ");
    else
      Serial.print(" ");
    Serial.print(local_servo_values[i]);
  }
  Serial.print(" ");
  Serial.print(chancol, HEX);
  Serial.print(fail_safe_mode ? " FAILSAFE " : " OK ");
  Serial.print(" O ");
  Serial.print(local_other_read_packets);
  Serial.print(" I ");
  Serial.print(local_invalid_read_packets);
  Serial.print(" (");
  Serial.print(100.0 * (float)local_invalid_read_packets / (float)local_this_read_packets);
  Serial.print(" %) T ");
  Serial.print(" %) T ");
  Serial.print(local_this_read_packets);
  uint32_t now_millis = millis();
  if (last_packets != 0)
  {
    Serial.print(" ");
    float delta_packets = local_this_read_packets - last_packets;
    float delta_millis = now_millis - last_millis;
    float hz = 1000.0 * delta_packets / delta_millis;
    Serial.print(hz);
    Serial.print(" Hz");
  }
  last_millis = now_millis;
  last_packets = local_this_read_packets;
  Serial.println();
  Serial.flush();
#else
  sbus_frame_counter++;
  if (sbus_frame_counter > 3)
    sbus_frame_counter = 0;
  sbus_frame_t sbus_frame;
  SBUS.send_frame(SBUSClass::build_sbus_frame(
      &sbus_frame,                                          // frame
      local_servo_values, NUMBER_OF_CHANNELS,               // channels
      fail_safe_mode, fail_safe_mode, sbus_frame_counter)); // status
#endif
}

void check_fail_safe_mode()
{
  // we are accessing shared variables so disable interrupts for now
  noInterrupts();
  if (!fail_safe_mode && (millis() - last_packet_read_time > FAIL_SAFE_TIME_MILLIS))
  {
    for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
      servo_values[i] = fail_safe_servo_values[i];
    fail_safe_mode = true;
    digitalWrite(LED_PIN, LED_OFF);
    set_chan(-1); // re-init but do not set channel
  }
  interrupts();
}

void loop()
{
  if (binding)
    binding_mode();
  else
  {
    check_fail_safe_mode();
    output_servo_values();
#ifdef DEBUG
    delay(100); // to give time to serial output
#else
    delay(6);
#endif
  }
}
