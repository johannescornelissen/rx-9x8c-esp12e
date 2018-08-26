// esp-12e code for 9x rx simulation with sbus output
// used a lot of code from midelic on RCgroups.com

#include "SPI.h" // use own extended spi lib
#include <EEPROM.h>

// config
#define DEBUG

// io
/*
  SPI GPIO13  MOSI  WHITE   R   YELLOW  A7105 DIO
      GPIO12  MISO  YELLOW  -   YELLOW  A7105 DIO
      GPIO14  CLK   GREEN   -   GREEN   A7105 CK
      GPIO15  EN    BROWN   -   WHITE   A7105 CS

  STA GPIO16        BROWN   -   BROWN   A7105 GIO1

  BND GPIO4         BLUE    R   Vcc/GND

  my 9x id 00035D9B

  latency taranis x9d:  23 ms    updates 111hz  sbus
  latency tgy i6s:      15 ms    updates 130hz  ibus

*/
#define LED_PIN 2 // GPIO2
#define LED_ON LOW
#define LED_OFF HIGH

#define BIND_PIN 4 // GPIO4
#define GIO_PIN 5  // GPIO5

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

// shared between normal code and interrupt handlers
volatile int chancol = 0;
volatile int this_read_packets = 0;
volatile int other_read_packets = 0;
volatile unsigned long last_packet_read_time = 0;
volatile bool fail_safe_mode = false;

// rx
#define NUMBER_OF_CHANNELS 8
volatile int servo_values[NUMBER_OF_CHANNELS]; // also shared between normal code and interrupt handlers

int fail_safe_servo_values[NUMBER_OF_CHANNELS] = {1500, 1500, 900, 1500, 1500, 1500, 1500, 1500}; // 9x: throttle 900 rest middel to signal failsafe
#define FAIL_SAFE_TIME_MILLIS 1500

//int missed_packets = 0;
//uint16_t nopacket = 0;
//int failsafeCnt = 0;

#define txid32 (txid[0] + (txid[1] << 8) + (txid[2] << 16) + (txid[3] << 24))

void setup()
{
  // initialize non-A7105
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);
  // debug console
#if defined(DEBUG)
  Serial.begin(74880); // esp default baud rate
  Serial.println("init");
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
  // write id
  SPI.writeb(0x00, 0x00); // reset
  SPI.writeid(0x06, 0x5475c52A);
#if defined(DEBUG)
  // read id
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

void binding_mode()
{
#if defined(DEBUG)
  Serial.println("entering binding mode");
#endif
  int counter = 0;
  SPI.strobe(0xa0);
  SPI.strobe(0xf0);
  SPI.writeb(0x0f, 0x00); // listen on channel 0 for binding
  SPI.strobe(0xc0);
  while (1)
  {
    delay(10);
    // switch led state on bit 3 of counter
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
          // first 4 bytes of packet are transmitter id
          for (int i = 0; i < 4; i++)
          {
            txid[i] = packet[i + 1];
            // store transmitter address part to eeprom
            EEPROM.write(txid_offset + i, txid[i]);
          }
          EEPROM.commit();
#if defined(DEBUG)
          Serial.print("found transmiter to bind to ");
          Serial.println(txid32, HEX); // convert 4 bytes via pointer to unsigned long
#endif
          // endless loop till reset (binding plug should be removed now)
          digitalWrite(LED_PIN, LED_ON);
          while (1)
            delay(10); // to avoid watchdog
        }
      }
      // restart
      SPI.strobe(0xa0);
      SPI.strobe(0xf0);
      SPI.writeb(0x0f, 0x00); // listen on channel 0 for binding
      SPI.strobe(0xc0);
    }
  }
}

void next_chan()
{
  SPI.strobe(0xA0);
  SPI.strobe(0xF0);
  SPI.writeb(0x0F, tx_channels[chanrow][chancol] - chanoffset - 1);
  SPI.strobe(0xC0);
  chancol = (chancol + 1) % 16;
}

void same_chan()
{
  SPI.strobe(0xA0);
  SPI.strobe(0xF0);
  SPI.strobe(0xC0);
}

/*
bool read_packet(uint8_t *packet, int size)
{
  if ((digitalRead(GIO_PIN) == 0) && (bitRead(SPI.readb(0x00), 5) == 0))
  {
    SPI.readdata(0x05, packet, size);
    return (packet[1] == txid[0]) && (packet[2] == txid[1]) && (packet[3] == txid[2]) && (packet[4] == txid[3]);
  }
  else
    return false;
}
*/

/*
void normal_mode()
{
  // try to read and process 1 valid packet
  next_chan();
  unsigned long start_millis = millis();
  uint8_t packet[21];
  bool fail_safe_active = false;
  int missed_loops = 0;
  while (!read_packet(packet, sizeof(packet)))
  {
    if (!fail_safe_active && (millis() - start_millis >= FAIL_SAFE_TIME_MILLIS))
    {
      // we go to failsafe
      fail_safe_active = true;
      digitalWrite(LED_PIN, LED_OFF);
      for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
        servo_values[i] = fail_safe_servo_values[i];
      output_servo_values(servo_values, NUMBER_OF_CHANNELS);
#ifdef DEBUG
      Serial.println(" FAILSAFE");
#endif
    }
    missed_loops++;
    // after too many missed loops try other channel but also give other processes time
    if (missed_loops % 500 == 0)
      missed_packets++;
    if (missed_loops % 5001 == 0)
    {
      yield();
      next_chan();
    }
  }
  read_packets++;
  if (fail_safe_active)
    digitalWrite(LED_PIN, LED_ON);
  // process packet to get channel values
  for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
  {
    int v = (packet[5 + (2 * i)] + 256 * packet[6 + (2 * i)]);
    if ((v >= 900) && (v <= 2200))
      servo_values[i] = v;
  }
  output_servo_values(servo_values, NUMBER_OF_CHANNELS);
#ifdef DEBUG
  if (read_packets % 50 == 0)
  {
    Serial.print(" ");
    Serial.print(chancol, HEX);
    Serial.print(" OK (");
    Serial.print(read_packets);
    Serial.print(", ");
    Serial.print(missed_packets);
    Serial.print(", ");
    Serial.print(missed_loops);
    Serial.println(")");
  }
#endif
  
}
*/

void handle_new_packet()
{
  // todo: read packet from rx
  // check packet valid flag
  if (bitRead(SPI.readb(0x00), 5) == 0)
  {
    uint8_t packet[21];
    SPI.readdata(0x05, packet, sizeof(packet));
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
    else {
      other_read_packets++;
      same_chan();
    }
  }
  else
  {
    same_chan();
  }
}

void output_servo_values()
{
  // copy servo values to local copy while interrupts are disabled
  int local_servo_values[NUMBER_OF_CHANNELS];
  int local_this_read_packets;
  int local_other_read_packets;
  // safe region
  noInterrupts();
  for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
    local_servo_values[i] = servo_values[i];
  local_this_read_packets = this_read_packets;
  local_other_read_packets = other_read_packets;
  interrupts();

  // todo: output safe servo values on sbus/ppm/ibus..

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
  if (fail_safe_mode)
  {
    Serial.print(" FAILSAFE ");
  }
  else
  {
    Serial.print(" OK ");
  }
  Serial.print(" O ");
  Serial.print(local_other_read_packets);
  Serial.print(" T ");
  Serial.println(local_this_read_packets);
  Serial.flush();
#endif
}

void check_fail_safe_mode()
{
  if (!fail_safe_mode && (millis() - last_packet_read_time > FAIL_SAFE_TIME_MILLIS))
  {
    noInterrupts();
    for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
      servo_values[i] = fail_safe_servo_values[i];
    fail_safe_mode = true;
    digitalWrite(LED_PIN, LED_OFF);
    
    same_chan();
    interrupts();
  }
}

void loop()
{
  if (binding)
    binding_mode();
  else
  {
    check_fail_safe_mode();
    output_servo_values();
    
  }
}
