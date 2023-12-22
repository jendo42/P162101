#include <Wire.h>
#include <SPI.h>
#include <limits.h>

#include "P162101.h"
#include "MCP79410.h"
#include "font3x5.h"
#include "font3x5_2.h"
#include "gamma.h"

#define STATUS_OK         0
#define STATUS_EBATTERY   1
#define STATUS_ERTC       2
#define STATUS_ESHT40     4

enum class SEQ {
  WARMUP,
  INTRO_S0,
  INTRO_S1,
  IDLE,
};

// system state variables
static int init_status = STATUS_OK;
static SEQ init_sequence = SEQ::WARMUP;
static uint32_t msg_timer = 0;
static const char *msg_text = NULL;
static char str_buffer[32];
static int light_sensor = 0;
static int led_power = 255;
static int btn_press[4];
static int display;

// LED matrix control variables
static int col = 0;
static uint64_t frame_cnt = 0;
static uint8_t buffers[2][MAT_COLS] = { 0 };
static uint8_t *frontbuffer = &buffers[0][0];
static uint8_t *backbuffer = &buffers[1][0];

static inline uint16_t rotl16(uint16_t n, unsigned int c)
{
  const unsigned int mask = (CHAR_BIT*sizeof(n) - 1);
  c &= mask;
  return (n<<c) | (n>>( (-c)&mask ));
}

static inline uint16_t rotr16(uint16_t n, unsigned int c)
{
  const unsigned int mask = (CHAR_BIT*sizeof(n) - 1);
  c &= mask;
  return (n>>c) | (n<<( (-c)&mask ));
}

uint8_t bcdPack(uint8_t x)
{
  return 10 * (x >> 4) + (x & 0xF);
}

uint8_t bcdUnpack(uint8_t x)
{
  return ((x / 10) << 4) | (x % 10);
}

void nibble2hex(uint8_t value, char *str)
{
  uint8_t c = value & 0xF;
  c += (c >= 10) ? 'A' : '0';
  str[0] = c;
  str[1] = 0;
}

void byte2hex(uint8_t value, char *str)
{
  nibble2hex(value >> 4, str);
  nibble2hex(value, str + 1);
}

// WARNING: this does not terminate string with '\0'
void value2str(int value, char *str)
{
  bool minus = false;
  if (value < 0) {
    minus = true;
    value = -value;
    str[0] = '-';
  } else {
    int digit = (value / 1000) % 10;
    str[0] = digit ? digit + '0' : ' ';
  }
  str[1] = (value /  100) % 10 + '0';
  str[2] = (value /   10) % 10 + '0';
  str[3] = '.';
  str[4] = (value /    1) % 10 + '0';
}

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
  shiftOut(GPIO_SPI_MOSI_BIT, GPIO_SPI_SCK_BIT, LSBFIRST, colData);
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

uint8_t glyphWidth(char ch)
{
  return font3x5_2_offsets[ch - ' '][1] + 1;
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
  if (init_status & STATUS_ERTC) {
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
  if (init_status & STATUS_ERTC) {
    return 0;
  }
  Wire.beginTransmission(0x6F);
  Wire.write(address);
  Wire.write(buffer, size);
  return Wire.endTransmission();
}

bool readClock(char *str)
{
  uint8_t reg[3];
  if (readRTC(MCP79410_RTCSEC, reg, sizeof(reg))) {
    str[0] = ((reg[2] >> 4) & 0x03) + '0';
    str[1] = ((reg[2] >> 0) & 0x0F) + '0';
    str[2] = ':';
    str[3] = ((reg[1] >> 4) & 0x07) + '0';
    str[4] = ((reg[1] >> 0) & 0x0F) + '0';
    str[5] = ':';
    str[6] = ((reg[0] >> 4) & 0x07) + '0';
    str[7] = ((reg[0] >> 0) & 0x0F) + '0';
    str[8] = 0;
    return true;
  }

  return false;
}

bool readClockFake(char *str)
{
  static uint8_t regx[3];
  static int cnt;
  if (++cnt >= 2) {
    cnt = 0;
    if (++regx[0] >= 60) {
      regx[0] = 0;
      if (++regx[1] >= 60) {
        regx[1] = 0;
        if (++regx[2] >= 24) {
          regx[2] = 0;
        }
      }
    }
  }

  uint8_t reg[3];
  for (int i = 0; i < sizeof(reg); i++) {
    reg[i] = bcdUnpack(regx[i]);
  }

  str[0] = ((reg[2] >> 4) & 0x03) + '0';
  str[1] = ((reg[2] >> 0) & 0x0F) + '0';
  str[2] = ':';
  str[3] = ((reg[1] >> 4) & 0x07) + '0';
  str[4] = ((reg[1] >> 0) & 0x0F) + '0';
  str[5] = ':';
  str[6] = ((reg[0] >> 4) & 0x07) + '0';
  str[7] = ((reg[0] >> 0) & 0x0F) + '0';
  str[8] = 0;
  return true;
}

// temperature is in deg. Celsius * 10
// (one decimal place fixed point)
bool readSHT40(int *t, int *rh)
{
  if (init_status & STATUS_ESHT40) {
    return false;
  }
  static uint32_t lastCommandTime = 0;
  uint8_t buff[6];
  uint32_t elapsed = millis() - lastCommandTime;
  if (elapsed >= 10) {
    // read data from sensor
    if (Wire.requestFrom(0x44, sizeof(buff)) != sizeof(buff)) {
      return false;
    }
    Wire.readBytes(buff, sizeof(buff));
    if (t) {
      int val = (buff[0] << 8) | buff[1];
      *t = -450 + 1750 * val / 65535;
    }
    if (rh) {
      int val = (buff[3] << 8) | buff[4];
      int lrh = -60 + 1250 * val / 65535;
      if (lrh > 1000) {
        lrh = 1000;
      }
      if (lrh < 0) {
        lrh = 0;
      }
      *rh = lrh;
    }

    // we can send command now
    Wire.beginTransmission(0x44);
    Wire.write(0xFD);
    Wire.endTransmission();
    lastCommandTime = millis();
  }

  return true;
}

bool readTemperature(char *str)
{
  int t;
  if (readSHT40(&t, NULL)) {
    value2str(t, str);
    str[5] = ' ';
    str[6] = '`';
    str[7] = 'C';
    str[8] = 0;
    return true;
  }

  return false;
}

bool readHumidity(char *str)
{
  int rh;
  if (readSHT40(NULL, &rh)) {
    value2str(rh, str);
    str[5] = ' ';
    str[6] = '%';
    str[7] = 0;
    return true;
  }

  return false;
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
  pinMode(GPIO_BOOT0_BIT, INPUT_PULLDOWN);

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
      init_status |= STATUS_EBATTERY;
    }
  } else {
    init_status |= STATUS_ERTC;
  }

  // initialize SHT40
  readSHT40(NULL, NULL);
  delay(11);
  if (!readSHT40(NULL, NULL)) {
    init_status |= STATUS_ESHT40;
  }

  // initialize light sensor value
  light_sensor = analogRead(GPIO_LIGHT_SENSE_BIT);

  // show error
  if (init_status != STATUS_OK) {
    str_buffer[0] = 'E';
    byte2hex(init_status, str_buffer + 1);
    showMessage(str_buffer, 128);
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


void drawClock(char *str)
{
  static char lastClock[9];
  static bool changed[8];
  static uint16_t image[MAT_COLS];
  static int animFrame = -1;

  if (animFrame == -1) {
    if (strcmp(lastClock, str) == 0) {
      drawText(1, 1, str);
      return;
    }

    // detect string changes
    if (strlen(str) != strlen(lastClock)) {
      strcpy(lastClock, str);
      drawText(1, 1, str);
      return;
    }

    for (int i = 0; i < 8; i++) {
      changed[i] = lastClock[i] != str[i];
      lastClock[i] = str[i];
    }

    // render new screen, save it to bitmap
    // combine old framebuffer with new
    drawText(1, 1, lastClock);
    for (int i = 0; i < MAT_COLS; i++) {
      image[i] = (frontbuffer[i]) | (backbuffer[i] << 7);
    }
  }

  ++animFrame;
  int animShift = animFrame >> 2;

  // shift-in new image, only glyphs that are changed
  int x = 1;
  for (int i = 0; i < 8; i++) {
    uint8_t w = glyphWidth(lastClock[i]);
    for (uint8_t j = 0; j < w; j++) {
      uint8_t a = x + j;
      if (changed[i]) {
        backbuffer[a] = rotl16(image[a], animShift + 2);
      } else {
        backbuffer[a] = image[a] >> 7;
      }
    }
    x += w;
  }

  if (animShift >= 7) {
    animFrame = -1;
  }
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

    // allow updating brightness after 16 frames,
    // when the value stabilize
    if (frame_cnt >= 16) {
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

    bool button_pressed = false;
    if (digitalRead(GPIO_B1_BIT) == LOW) {
      if (btn_press[0] == 0) {
        uint8_t data;
        readRTC(MCP79410_RTCHOUR, &data, 1);
        data = (data & 0xC0) | bcdUnpack((bcdPack(data & 0x3F) + 1) % 24);
        writeRTC(MCP79410_RTCHOUR, &data, 1);
        button_pressed = true;
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
        button_pressed = true;
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
        button_pressed = true;
      }
      if (++btn_press[2] >= 64) {
        btn_press[2] = 0;
      }
    } else {
      btn_press[2] = 0;
    }
    if (digitalRead(GPIO_BOOT0_BIT) == HIGH) {
      if (btn_press[3] == 0) {
        ++display;
        button_pressed = true;
      }
      if (++btn_press[3] >= 64) {
        btn_press[3] = 0;
      }
    } else {
      btn_press[3] = 0;
    }

    int seq = frame_cnt & 0x1;
    if (seq == 0 || button_pressed) {
      // execute this code every 4 frames
      clearScreen(0);
      if (init_sequence < SEQ::IDLE) {
        // intro animation sequence
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
        // show message with timeout set with showMessage()
        --msg_timer;
        if (frame_cnt & 16) {
          drawText(1, 1, msg_text);
        }
      } else {
        // do only each 64 frames
        if ((frame_cnt & 0x0F) == 0 || button_pressed) {
          switch (display) {
            case 0:
              if (!readClock(str_buffer)) {
                strcpy(str_buffer, "RTC!");
              }
              break;
            case 1:
              if (!readTemperature(str_buffer)) {
                strcpy(str_buffer, "SHT40!");
              }
              break;
            case 2:
              if (!readHumidity(str_buffer)) {
                strcpy(str_buffer, "SHT40!");
              }
              break;
            default:
              display = 0;
              break;
          }
        }

        if (display == 0) {
          // animate clock
          drawClock(str_buffer);
        } else {
          //snprintf(str_buffer, sizeof(str_buffer), "%u", display);
          drawText(1, 1, str_buffer);
        }
      }

      swapBuffers();
    }
  }
}
