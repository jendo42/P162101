#include <Wire.h>
#include <SPI.h>

#include "P162101.h"
#include "MCP79410.h"
#include "font3x5.h"
#include "font3x5_2.h"
#include "gamma.h"

enum class STATUS {
  OK,
  EBATTERY,
  ERTC,
};

enum class SEQ {
  WARMUP,
  INTRO_S0,
  INTRO_S1,
  IDLE,
};

// system state variables
static STATUS init_status = STATUS::OK;
static SEQ init_sequence = SEQ::WARMUP;
static uint32_t msg_timer = 0;
static const char *msg_text = NULL;
static char str_buffer[32];
static int light_sensor = 0;
static int led_power = 255;
static int btn_press[3];

// LED matrix control variables
static int col = 0;
static uint64_t frame_cnt = 0;
static uint8_t buffers[2][MAT_COLS] = { 0 };
static uint8_t *frontbuffer = &buffers[0][0];
static uint8_t *backbuffer = &buffers[1][0];

void showMessage(const char *text, uint32_t frames)
{
  msg_text = text;
  msg_timer = frames;
}

void enableCol(int address)
{
  // mapping of HW adresses of columns
  // total 29 columns
  static const uint8_t colMap[] = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 15, 14, 13, 12, 11,
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 12, 11
  };

  // extract bus address
  int busAddr = colMap[address % sizeof(colMap)];

  // logic is negated (active-low signals)
  int chipSelect = (address & 0x10) ? bit(GPIO_N_MAT_ADDRL_NBIT) : bit(GPIO_N_MAT_ADDRH_NBIT);

  // extract bits that will be set and reset
  int bitsSet = (chipSelect | (busAddr << GPIO_MAT_ADDR0_NBIT));
  int bitsReset = ~bitsSet & MAT_CONTROL_MASK;
  // using BSRR (bit set-reset register)
  // to quickly setup decoders
  GPIOA->BSRR = (bitsReset << 16) | (bitsSet);

  // NOTE: 74HC154 takes some time to propagate input signals to output,
  // during this time output of 74HC595 should be turned off to prevent
  // visual artifacts
}

void disableCols()
{
  // set ADDRL and ADDRH to disable column decoders
  GPIOA->BSRR = bit(GPIO_N_MAT_ADDRL_BIT) | bit(GPIO_N_MAT_ADDRH_BIT);
}

void enableRows()
{
  digitalWrite(GPIO_N_MAT_ROE_BIT, LOW);
}

void disableRows()
{
  digitalWrite(GPIO_N_MAT_ROE_BIT, HIGH);
}

void transmitCol(uint8_t colData)
{
  // send display data to 74HC595
  digitalWrite(GPIO_SPI_NSS_BIT, LOW);
  for (int i = 0; i < 8; i++) {
    digitalWrite(GPIO_SPI_MOSI_BIT, (colData & 1) ? HIGH : LOW);
    digitalWrite(GPIO_SPI_SCK_BIT, LOW);
    colData >>= 1;
    digitalWrite(GPIO_SPI_SCK_BIT, HIGH);
  }
  digitalWrite(GPIO_SPI_NSS_BIT, HIGH);
}

void transmitColSPI(uint8_t colData)
{
  digitalWrite(GPIO_SPI_NSS_BIT, LOW);
  SPI.transfer(colData);
  digitalWrite(GPIO_SPI_NSS_BIT, HIGH);
}

void clearScreen(uint8_t pattern)
{
  memset(backbuffer, pattern, MAT_COLS);
}

void drawText(int x0, int y0, const char *text)
{
  while (*text) {
    char ch = *text - ' ';
    uint8_t o = font3x5_2_offsets[ch][0];
    uint8_t l = font3x5_2_offsets[ch][1];
    for (uint8_t x = 0; x < l; x++) {
      uint8_t g = font3x5_2_bitmap[o + x];
      if (y0 > 0) {
        g <<= y0;
      } else if (y0 < 0) {
        g >>= -y0;
      }
      if (x0 >= 0 && x0 < MAT_COLS) {
        backbuffer[x0] ^= g;
      }
      ++x0;
    }
    ++x0;
    ++text;
  }
}

void swapBuffers()
{
  uint8_t *x = frontbuffer;
  frontbuffer = backbuffer;
  backbuffer = x;
}

size_t readRTC(uint8_t address, uint8_t *buffer, size_t size)
{
  if (init_status == STATUS::ERTC) {
    return 0;
  }
  Wire.beginTransmission(0x6F);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(0x6F, size);
  return Wire.readBytes(buffer, size);
}

uint8_t writeRTC(uint8_t address, uint8_t *buffer, size_t size)
{
  if (init_status == STATUS::ERTC) {
    return 0;
  }
  Wire.beginTransmission(0x6F);
  Wire.write(address);
  Wire.write(buffer, size);
  return Wire.endTransmission();
}

void setup()
{
  // TODO: wait for voltage stabilize

  // initialize pins
  pinMode(GPIO_MAT_ADDR0_BIT, OUTPUT);
  pinMode(GPIO_MAT_ADDR1_BIT, OUTPUT);
  pinMode(GPIO_MAT_ADDR2_BIT, OUTPUT);
  pinMode(GPIO_MAT_ADDR3_BIT, OUTPUT);
  pinMode(GPIO_N_MAT_ADDRL_BIT, OUTPUT);
  pinMode(GPIO_N_MAT_ADDRH_BIT, OUTPUT);

  pinMode(GPIO_SPI_NSS_BIT, OUTPUT);
  pinMode(GPIO_SPI_SCK_BIT, OUTPUT);
  pinMode(GPIO_SPI_MOSI_BIT, OUTPUT);
  pinMode(GPIO_N_MAT_ROE_BIT, OUTPUT);

  pinMode(GPIO_B1_BIT, INPUT_PULLUP);
  pinMode(GPIO_B2_BIT, INPUT_PULLUP);
  pinMode(GPIO_B3_BIT, INPUT_PULLUP);

  pinMode(GPIO_SDA_BIT, OUTPUT_OPEN_DRAIN);
  pinMode(GPIO_SCL_BIT, OUTPUT_OPEN_DRAIN);

  pinMode(GPIO_LIGHT_SENSE_BIT, INPUT);

  digitalWrite(GPIO_N_MAT_ROE_BIT, HIGH);
  digitalWrite(GPIO_N_MAT_ADDRL_BIT, HIGH);
  digitalWrite(GPIO_N_MAT_ADDRH_BIT, HIGH);
  digitalWrite(GPIO_SCL_BIT, HIGH);
  digitalWrite(GPIO_SCL_BIT, HIGH);
  digitalWrite(GPIO_SDA_BIT, HIGH);

  analogReference(AR_DEFAULT);

  //SPI.setMOSI(GPIO_SPI_MOSI_BIT);
  //SPI.setSCLK(GPIO_SPI_SCK_BIT);
  //SPI.setSSEL(GPIO_SPI_NSS_BIT);
  //SPI.begin();
  //SPI.beginTransaction(SPISettings(6000000, LSBFIRST, SPI_MODE0));

  Wire.setSDA(GPIO_SDA_BIT);
  Wire.setSCL(GPIO_SCL_BIT);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(10);

  // initialize RTC
  uint8_t reg;
  if (readRTC(MCP79410_RTCWKDAY, &reg, 1)) {
    if (~reg & bit(MCP79410_RTCWKDAY_OSCRUN)) {
      bitSet(reg, MCP79410_RTCWKDAY_VBATEN);
      bitClear(reg, MCP79410_RTCWKDAY_PWRFAIL);
      writeRTC(MCP79410_RTCWKDAY, &reg, 1);
      reg = 0x80;
      writeRTC(MCP79410_RTCSEC, &reg, 1);
      writeRTC(MCP79410_RTCHOUR, &reg, 1);
      init_status = STATUS::EBATTERY;
    }
  } else {
    init_status = STATUS::ERTC;
  }
  //if (checksum != (0xFEEDC0DE - 0xBAADBEEF)) {
  //  init_status = STATUS::ERTC;
  //}

  switch (init_status) {
    default:
    case STATUS::OK:
      break;
    case STATUS::EBATTERY:
      showMessage("BAT!", 16);
      break;
    case STATUS::ERTC:
      showMessage("RTC!", 16);
      break;
  }
}

void update_brightness(int light_min, int light_max, int power_min, int power_max)
{
  int r = light_max - light_min;
  int i = light_sensor - light_min;
  if (i > r) {
    i = r;
  }
  if (i < 0) {
    i = 0;
  }
  
  int r2 = power_max - power_min;
  led_power = power_min + (r2 * i / r);
}

uint8_t bcdPack(uint8_t x)
{
  return 10 * (x >> 4) + (x & 0xF);
}

uint8_t bcdUnpack(uint8_t x)
{
  return ((x / 10) << 4) | (x % 10);
}

void loop() {

  enableCol(col);
  transmitCol(frontbuffer[col]);

  if (led_power > 255) {
    led_power = 255;
  }
  if (led_power < 0) {
    led_power = 0;
  }

  int time_on =  250 * gamma_lut_8to8[led_power] / 255;
  int time_off = 250 - time_on;

  if (time_on) {
    enableRows();
    delayMicroseconds(time_on);
  }

  disableRows();
  disableCols();
  
  if (time_off) {
    delayMicroseconds(time_off);
  }

  if (++col >= MAT_COLS) {
    col = 0;
    frame_cnt++;

    // moving average (64 frames delay)
    int light_current = analogRead(GPIO_LIGHT_SENSE_BIT);
    light_sensor += (light_current - light_sensor) >> 5;
    if (frame_cnt >= 128) {
      // update brightness
      // 25..250 linear
      update_brightness(25, 200, 64, 255);
      if (init_sequence == SEQ::WARMUP) {
        init_sequence = SEQ::INTRO_S0;
      }
    }

    if (init_sequence == SEQ::WARMUP) {
      return;
    }

    if (digitalRead(GPIO_B1_BIT) == LOW) {
      if (btn_press[0] == 0) {
        uint8_t data;
        readRTC(MCP79410_RTCHOUR, &data, 1);
        data = (data & 0xC0) | bcdUnpack((bcdPack(data & 0x3F) + 1) % 24);
        writeRTC(MCP79410_RTCHOUR, &data, 1);
      }
      if (++btn_press[0] >= 64) {
        btn_press[0] = 0;
      }
    } else {
      btn_press[0] = 0;
    }
    if (digitalRead(GPIO_B2_BIT) == LOW) {
      if (btn_press[1] == 0) {
        uint8_t data;
        readRTC(MCP79410_RTCMIN, &data, 1);
        data = bcdUnpack((bcdPack(data) + 1) % 60);
        writeRTC(MCP79410_RTCMIN, &data, 1);
      }
      if (++btn_press[1] >= 64) {
        btn_press[1] = 0;
      }
    } else {
      btn_press[1] = 0;
    }
    if (digitalRead(GPIO_B3_BIT) == LOW) {
      if (btn_press[2] == 0) {
        uint8_t data;
        readRTC(MCP79410_RTCSEC, &data, 1);
        data = (data & 0x80);
        writeRTC(MCP79410_RTCSEC, &data, 1);
      }
      if (++btn_press[2] >= 64) {
        btn_press[2] = 0;
      }
    } else {
      btn_press[2] = 0;
    }
    
    int seq = frame_cnt & 0x3;
    if (seq == 0) {
      clearScreen(0);
      if (init_sequence < SEQ::IDLE) {
        static int x = 0;
        if (init_sequence == SEQ::INTRO_S1) {
          clearScreen(0xFF);
        }
        for (int i = 0; i < x; i++) {
          backbuffer[i] = init_sequence == SEQ::INTRO_S1 ? 0x00 : 0xFF;          
        }
        if (++x >= MAT_COLS) {
          x = 0;
          init_sequence = (SEQ)((int)init_sequence + 1);
        }
      } else if (msg_timer && msg_text) {
        --msg_timer;
        drawText(1, 1, msg_text);
      } else {
        if ((frame_cnt & 0x1F) == 0) {
          // update clock
          uint8_t reg[3];
          if (readRTC(MCP79410_RTCSEC, &reg[0], sizeof(reg))) {
            str_buffer[0] = ((reg[2] >> 4) & 0x03) + '0';
            str_buffer[1] = ((reg[2] >> 0) & 0x0F) + '0';
            str_buffer[2] = ':';
            str_buffer[3] = ((reg[1] >> 4) & 0x07) + '0';
            str_buffer[4] = ((reg[1] >> 0) & 0x0F) + '0';
            str_buffer[5] = ':';
            str_buffer[6] = ((reg[0] >> 4) & 0x07) + '0';
            str_buffer[7] = ((reg[0] >> 0) & 0x0F) + '0';
            str_buffer[8] = 0;
          } else {
            strcpy(str_buffer, "99:99:99");
          }
        }

        //snprintf(str_buffer, sizeof(str_buffer), "%u", led_power);
        drawText(1, 1, str_buffer);
      }
      swapBuffers();
    }
  }
}
