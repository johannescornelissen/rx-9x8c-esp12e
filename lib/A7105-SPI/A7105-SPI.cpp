/*
  based on SPI.cpp - SPI library for esp8266

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  added some extra functions for addressing A7105
*/

#include "A7105-SPI.h"
#include "HardwareSerial.h"

#define SPI_PINS_HSPI 0         // Normal HSPI mode (MISO = GPIO12, MOSI = GPIO13, SCLK = GPIO14);
#define SPI_PINS_HSPI_OVERLAP 1 // HSPI Overllaped in spi0 pins (MISO = SD0, MOSI = SDD1, SCLK = CLK);

#define SPI_OVERLAP_SS 0

typedef union {
  uint32_t regValue;
  struct
  {
    unsigned regL : 6;
    unsigned regH : 6;
    unsigned regN : 6;
    unsigned regPre : 13;
    unsigned regEQU : 1;
  };
} spiClk_t;

SPIClass::SPIClass()
{
  useHwCs = false;
  pinSet = SPI_PINS_HSPI;
}

void SPIClass::begin()
{
  switch (pinSet)
  {
  case SPI_PINS_HSPI_OVERLAP:
    IOSWAP |= (1 << IOSWAP2CS);
    //SPI0E3 |= 0x1; This is in the MP3_DECODER example, but makes the WD kick in here.
    SPI1E3 |= 0x3;

    setHwCs(true);
    break;
  case SPI_PINS_HSPI:
  default:
    pinMode(SCK, SPECIAL);  ///< GPIO14
    pinMode(MISO, SPECIAL); ///< GPIO12
    pinMode(MOSI, SPECIAL); ///< GPIO13
    break;
  }

  SPI1C = 0;
  setFrequency(1000000); ///< 1MHz
  SPI1U = SPIUMOSI | SPIUDUPLEX | SPIUSSE;
  SPI1U1 = (7 << SPILMOSI) | (7 << SPILMISO);
  SPI1C1 = 0;
}

void SPIClass::end()
{
  switch (pinSet)
  {
  case SPI_PINS_HSPI:
    pinMode(SCK, INPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, INPUT);
    if (useHwCs)
    {
      pinMode(SS, INPUT);
    }
    break;
  case SPI_PINS_HSPI_OVERLAP:
    IOSWAP &= ~(1 << IOSWAP2CS);
    if (useHwCs)
    {
      SPI1P |= SPIPCS1DIS | SPIPCS0DIS | SPIPCS2DIS;
      pinMode(SPI_OVERLAP_SS, INPUT);
    }
    break;
  }
}

void SPIClass::setHwCs(bool use)
{
  switch (pinSet)
  {
  case SPI_PINS_HSPI:
    if (use)
    {
      pinMode(SS, SPECIAL); ///< GPIO15
      SPI1U |= (SPIUCSSETUP | SPIUCSHOLD);
    }
    else
    {
      if (useHwCs)
      {
        pinMode(SS, INPUT);
        SPI1U &= ~(SPIUCSSETUP | SPIUCSHOLD);
      }
    }
    break;
  case SPI_PINS_HSPI_OVERLAP:
    if (use)
    {
      pinMode(SPI_OVERLAP_SS, FUNCTION_1); // GPI0 to SPICS2 mode
      SPI1P &= ~SPIPCS2DIS;
      SPI1P |= SPIPCS1DIS | SPIPCS0DIS;
      SPI1U |= (SPIUCSSETUP | SPIUCSHOLD);
    }
    else
    {
      if (useHwCs)
      {
        pinMode(SPI_OVERLAP_SS, INPUT);
        SPI1P |= SPIPCS1DIS | SPIPCS0DIS | SPIPCS2DIS;
        SPI1U &= ~(SPIUCSSETUP | SPIUCSHOLD);
      }
    }
    break;
  }

  useHwCs = use;
}
/**
   calculate the Frequency based on the register value
   @param reg
   @return
*/
static uint32_t ClkRegToFreq(spiClk_t *reg)
{
  return (ESP8266_CLOCK / ((reg->regPre + 1) * (reg->regN + 1)));
}

void SPIClass::setFrequency(uint32_t freq)
{
  static uint32_t lastSetFrequency = 0;
  static uint32_t lastSetRegister = 0;

  if (freq >= ESP8266_CLOCK)
  {
    setClockDivider(0x80000000);
    return;
  }

  if (lastSetFrequency == freq && lastSetRegister == SPI1CLK)
  {
    // do nothing (speed optimization)
    return;
  }

  const spiClk_t minFreqReg = {0x7FFFF000};
  uint32_t minFreq = ClkRegToFreq((spiClk_t *)&minFreqReg);
  if (freq < minFreq)
  {
    // use minimum possible clock
    setClockDivider(minFreqReg.regValue);
    lastSetRegister = SPI1CLK;
    lastSetFrequency = freq;
    return;
  }

  uint8_t calN = 1;

  spiClk_t bestReg = {0};
  int32_t bestFreq = 0;

  // find the best match
  while (calN <= 0x3F)
  { // 0x3F max for N

    spiClk_t reg = {0};
    int32_t calFreq;
    int32_t calPre;
    int8_t calPreVari = -2;

    reg.regN = calN;

    while (calPreVari++ <= 1)
    { // test different variants for Pre (we calculate in int so we miss the decimals, testing is the easyest and fastest way)
      calPre = (((ESP8266_CLOCK / (reg.regN + 1)) / freq) - 1) + calPreVari;
      if (calPre > 0x1FFF)
      {
        reg.regPre = 0x1FFF; // 8191
      }
      else if (calPre <= 0)
      {
        reg.regPre = 0;
      }
      else
      {
        reg.regPre = calPre;
      }

      reg.regL = ((reg.regN + 1) / 2);
      // reg.regH = (reg.regN - reg.regL);

      // test calculation
      calFreq = ClkRegToFreq(&reg);
      //os_printf("-----[0x%08X][%d]\t EQU: %d\t Pre: %d\t N: %d\t H: %d\t L: %d = %d\n", reg.regValue, freq, reg.regEQU, reg.regPre, reg.regN, reg.regH, reg.regL, calFreq);

      if (calFreq == (int32_t)freq)
      {
        // accurate match use it!
        memcpy(&bestReg, &reg, sizeof(bestReg));
        break;
      }
      else if (calFreq < (int32_t)freq)
      {
        // never go over the requested frequency
        if (abs(freq - calFreq) < abs(freq - bestFreq))
        {
          bestFreq = calFreq;
          memcpy(&bestReg, &reg, sizeof(bestReg));
        }
      }
    }
    if (calFreq == (int32_t)freq)
    {
      // accurate match use it!
      break;
    }
    calN++;
  }

  // os_printf("[0x%08X][%d]\t EQU: %d\t Pre: %d\t N: %d\t H: %d\t L: %d\t - Real Frequency: %d\n", bestReg.regValue, freq, bestReg.regEQU, bestReg.regPre, bestReg.regN, bestReg.regH, bestReg.regL, ClkRegToFreq(&bestReg));

  setClockDivider(bestReg.regValue);
  lastSetRegister = SPI1CLK;
  lastSetFrequency = freq;
}

void SPIClass::setClockDivider(uint32_t clockDiv)
{
  if (clockDiv == 0x80000000)
  {
    GPMUX |= (1 << 9); // Set bit 9 if sysclock required
  }
  else
  {
    GPMUX &= ~(1 << 9);
  }
  SPI1CLK = clockDiv;
}

inline void SPIClass::setDataBits(uint16_t bits)
{
  const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
  bits--;
  SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));
}

void SPIClass::strobe(uint8_t command)
{
  // same as write only highest 4 bits are transfered
  while (SPI1CMD & SPIBUSY)
  {
  }
  // reset to 8Bit mode
  setDataBits(4);
  SPI1W0 = command;
  SPI1CMD |= SPIBUSY;
  while (SPI1CMD & SPIBUSY)
  {
  }
}

void SPIClass::writeid(uint8_t address, uint32_t data)
{
  while (SPI1CMD & SPIBUSY)
  {
  }
  setDataBits(5 * 8);
  SPI1W0 = address | ((data & 0xff000000) >> 16) | (data & 0x00ff0000) | ((data & 0x0000ff00) << 16);
  SPI1W1 = data & 0xff;
  SPI1CMD |= SPIBUSY;
  while (SPI1CMD & SPIBUSY)
  {
  }
}

void SPIClass::writeb(uint8_t address, uint8_t data)
{
  while (SPI1CMD & SPIBUSY)
  {
  }
  setDataBits(2 * 8);
  SPI1W0 = address | (data << 8);
  SPI1CMD |= SPIBUSY;
  while (SPI1CMD & SPIBUSY)
  {
  }
}

uint32_t SPIClass::readid(uint8_t address)
{
  while (SPI1CMD & SPIBUSY)
  {
  }
  setDataBits(5 * 8);
  SPI1W0 = address | 0x40 | 0xffffff00; // 40=mark as read operation ff=mosi high during read
  SPI1W1 = 0xff;                        // ff=mosi high during read
  SPI1CMD |= SPIBUSY;
  while (SPI1CMD & SPIBUSY)
  {
  }
  return ((SPI1W0 & 0x0000ff00) << 16) | (SPI1W0 & 0x00ff0000) | ((SPI1W0 & 0xff000000) >> 16) | (SPI1W1 & 0xff);
}

uint8_t SPIClass::readb(uint8_t address)
{
  while (SPI1CMD & SPIBUSY)
  {
  }
  setDataBits(2 * 8);
  SPI1W0 = address | 0x40 | 0xff00; // 40=mark as read operation ff00=mosi high during read
  SPI1CMD |= SPIBUSY;
  while (SPI1CMD & SPIBUSY)
  {
  }
  return SPI1W0 >> 8;
}

void SPIClass::readdata(uint8_t address, uint8_t *data, int size)
{
  while (SPI1CMD & SPIBUSY)
  {
  }
  setDataBits((1 + size) * 8);
  // fill sending buffer
  // first ulong
  SPI1W0 = address | 0x40 | 0xffffff00; // 40=mark as read operation ff00=mosi high during read
  // rest of ulongs
  for (int l = 1; l <= (size / 4); l++)
  {
    SPI1W(l) = 0xffffffff;
  }
  // start transfer
  SPI1CMD |= SPIBUSY;
  while (SPI1CMD & SPIBUSY)
  {
  }
  // start fetching returned data at first data byte ie byte 1 of ulong 0
  int l = 0;
  int ib = 1;
  for (int i = 0; i < size; i++)
  {
    data[i] = (SPI1W(l) >> ib * 8);
    ib++;
    if (ib == 4)
    {
      ib = 0;
      l++;
    }
  }
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_SPI)
SPIClass SPI;
#endif
