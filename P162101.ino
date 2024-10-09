#include <Wire.h>
#include <SPI.h>

#include "P162101.h"
#include "MCP79410.h"
#include "SHT4X.h"
#include "font3x5.h"
#include "font3x5_2.h"
#include "gamma.h"
#include "utility.h"
#include "names.h"
#include "game.h"

#define STATUS_OK         0
#define STATUS_TEST       8

enum class SEQ {
  WARMUP,
  INTRO_S0,
  INTRO_S1,
  IDLE,
  DISPT1,
  DISPT2,
};

// system state variables
int init_status = STATUS_OK;

static SEQ init_sequence = SEQ::WARMUP;
static uint32_t msg_timer = 0;
static const char *msg_text = NULL;
static char str_buffer[64];
static char str_temp[64];
static char str_rh[64];
static int light_sensor = 0;
static int led_power = 255;
static bool clock_reload = false;
static bool autoscroll = false;
static int text_scroll = 0;

static int btn_press[4];
static bool btn_longpress[4];
const int btn_map[] = {
  GPIO_B1_BIT,
  GPIO_B2_BIT,
  GPIO_B3_BIT,
  GPIO_BOOT0_BIT,
};
const int btn_map_state[] = {
  LOW,
  LOW,
  LOW,
  HIGH,
};

static int display;
static int new_display;
static int clock_trim;
static int game_active;
static int autotransition_timeout = 1;

// LED matrix control variables
#define BUFFER_SIZE (MAT_COLS * 2)
static int col = 0;
static volatile uint64_t frame_cnt = 0;
static uint8_t buffers[2][BUFFER_SIZE] = { 0 };
static uint8_t *frontbuffer = &buffers[0][0];
static uint8_t *backbuffer = &buffers[1][0];
static uint32_t viewport_x = 0;
static HardwareTimer display_timer;

extern "C" void HardFault_Handler(void)
{
  while (1) {
    enableCol(14);
    transmitCol(0xEE);
    enableRows();
    for (uint32_t i = 0x800; i; i--) asm("NOP");
    disableRows();
    for (uint32_t i = 0x10000; i; i--) asm("NOP");
  }
}

void showMessage(const char *text, uint32_t frames)
{
  if (text && !text[0]) {
    text = NULL;
    frames = 0;
  }
  msg_text = text;
  msg_timer = frames;
}

void enableCol(int address)
{
  // mapping of HW adresses of columns
  // total 29 columns
  static const uint8_t colMap[] = {
     10,  9,  8,  7,  6,  5,  4,  3,  2,  1, 0, 11, 12, 13, 14, 15,
     10,  9,  8,  7,  6,  5,  4,  3,  2,  1, 0, 14, 15,
     // the old mapping for HW compatibility with prototype board
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 15, 14, 13, 12, 11,
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 12, 11
  };

  // extract bus address
  const int colMapSize = sizeof(colMap) / 2;
  int busAddr = colMap[address % colMapSize];

  // logic is negated (active-low signals)
  int chipSelect = (address & 0x10) ? bit(GPIO_N_MAT_ADDRL_NBIT) : bit(GPIO_N_MAT_ADDRH_NBIT);

  // extract bits that will be set and reset
  int bitsSet = (chipSelect | (busAddr << GPIO_MAT_ADDR0_NBIT)) | bit(GPIO_N_MAT_ROE_NBIT);
  int bitsReset = (~bitsSet & (MAT_COL_ADDR_MASK | MAT_COL_CTRL_MASK));

  // using BSRR (bit set-reset register)
  // to quickly setup decoders
  GPIOA->BSRR = (bitsReset << 16) | (bitsSet);
}

void disableCols()
{
  GPIOA->BSRR = bit(GPIO_N_MAT_ADDRL_NBIT) | bit(GPIO_N_MAT_ADDRH_NBIT);
}

void enableRows()
{
  GPIOA->BRR = bit(GPIO_N_MAT_ROE_NBIT);
}

void disableRows()
{
  // fast shutdown
  GPIOA->BSRR = bit(GPIO_N_MAT_ROE_NBIT);
  // clean data to prevent artefacts
  transmitCol(0);
}

void transmitCol(uint8_t colData)
{
  // send display data to 74HC595
  digitalWrite(GPIO_SPI_NSS_BIT, LOW);
  shiftOut(GPIO_SPI_MOSI_BIT, GPIO_SPI_SCK_BIT, LSBFIRST, colData);
  digitalWrite(GPIO_SPI_NSS_BIT, HIGH);
}

void clearScreen(uint8_t pattern)
{
  memset(backbuffer, pattern, BUFFER_SIZE);
}

void xorPixel(int x, int y)
{
  uint8_t p = 1 << y;
  if (x >= 0 && x < BUFFER_SIZE) {
    backbuffer[x] ^= p;
  }
}

bool getPixel(int x, int y)
{
  uint8_t p = 1 << y;
  if (x >= 0 && x < BUFFER_SIZE) {
    return backbuffer[x] & p;
  }
}

uint8_t glyphWidth(char ch)
{
  return font3x5_2_offsets[ch - ' '][1] + 1;
}

int drawText(int x0, int y0, const char *text)
{
  int w = 0;
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
      if (x0 >= 0 && x0 < BUFFER_SIZE) {
        backbuffer[x0] ^= g;
      }
      ++x0;
      ++w;
    }
    ++x0;
    ++w;
    ++text;
  }

  return w;
}

void swapBuffers()
{
  uint8_t *x = frontbuffer;
  frontbuffer = backbuffer;
  backbuffer = x;
}

bool readClock(char *str, int type)
{
  uint8_t reg[7];
  int day;
  if (readRTC(MCP79410_RTCSEC, reg, sizeof(reg))) {
    switch (type) {
      case 0:
        str[0] = ((reg[2] >> 4) & 0x03) + '0'; // h
        str[1] = ((reg[2] >> 0) & 0x0F) + '0';
        str[2] = ':';
        str[3] = ((reg[1] >> 4) & 0x07) + '0'; // m
        str[4] = ((reg[1] >> 0) & 0x0F) + '0';
        str[5] = ':';
        str[6] = ((reg[0] >> 4) & 0x07) + '0'; // s
        str[7] = ((reg[0] >> 0) & 0x0F) + '0';
        str[8] = 0;
        break;
      case 1:
        day = dayOfWeek(bcdPack(reg[6]), bcdPack(reg[5] & 0x1F), bcdPack(reg[4] & 0x3F)) - 1;
        strcpy(str, weekday_names_sk[day]);
        strupr(str);
        break;
      case 2:
        str[0] = ((reg[4] >> 4) & 0x03) + '0'; // d
        str[1] = ((reg[4] >> 0) & 0x0F) + '0';
        str[2] = '.';
        str[3] = ((reg[5] >> 4) & 0x01) + '0'; // m
        str[4] = ((reg[5] >> 0) & 0x0F) + '0';
        str[5] = '.';
        str[6] = ((reg[6] >> 4) & 0x0F) + '0'; // y
        str[7] = ((reg[6] >> 0) & 0x0F) + '0';
        str[8] = 0;
        break;
      case 3:
        strcpy(str, calendar_names_sk[bcdPack(reg[5] & 0x1F) - 1][bcdPack(reg[4] & 0x3F) - 1]);
        strupr(str);
        break;
      default:
        break;
    }
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

bool readTemperatureHumidity(char *temp_str, char *rh_str)
{
  static int rh;
  static int t;
  static int trimm_timer = 0;
  int c1, c2;
  if (readSHT40(&c1, &c2)) {
    if (!triggerSHT40()) {
      return false;
    }

    // moving average (4 frames delay)
    t += (c1 - t) >> 2;
    value2str(t, temp_str);
    temp_str[5] = ' ';
    temp_str[6] = '`';
    temp_str[7] = 'C';
    temp_str[8] = 0;

    // moving average (4 frames delay)
    rh += (c2 - rh) >> 2;
    value2str(rh, rh_str);
    rh_str[5] = ' ';
    rh_str[6] = '%';
    rh_str[7] = 0;

    // update clock trim
    if (++trimm_timer >= 127) {
      trimm_timer = 0;
      trimRTC(t);
    }
    return true;
  }

  return false;
}

bool test_mode()
{
  return init_status & STATUS_TEST;
}

void update_lightsensor()
{
  // moving average (64 frames delay)
  int light_current = analogRead(GPIO_LIGHT_SENSE_BIT);
  light_sensor += (light_current - light_sensor) >> 6;
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

  // clamp power
  if (led_power > 255) {
    led_power = 255;
  }
  if (led_power < 0) {
    led_power = 0;
  }

  int gamma = gamma_lut_8to8[led_power];
  display_timer.setCaptureCompare(1, gamma * 100 / 255, PERCENT_COMPARE_FORMAT);
}

void display_begin()
{
  int addr = (col + viewport_x) % BUFFER_SIZE;
  enableCol(col);
  transmitCol(frontbuffer[addr]);
  enableRows();
}

void display_end()
{
  // switch off LEDS
  disableRows();

  // update image data pointer
  if (++col >= MAT_COLS) {
    col = 0;
    ++frame_cnt;
    update_lightsensor();
    // allow updating brightness after 16 frames,
    // when the value stabilize
    if (frame_cnt >= 16) {
      // update brightness
      // 25..500 linear
      update_brightness(25, 500, 64, 255);
    }
  }
}

void setup()
{
  // wait for voltage stabilize
  delay(100);

  // horizontal LED control pins
  pinMode(GPIO_MAT_ADDR0_BIT, OUTPUT);
  pinMode(GPIO_MAT_ADDR1_BIT, OUTPUT);
  pinMode(GPIO_MAT_ADDR2_BIT, OUTPUT);
  pinMode(GPIO_MAT_ADDR3_BIT, OUTPUT);
  pinMode(GPIO_N_MAT_ADDRL_BIT, OUTPUT);
  pinMode(GPIO_N_MAT_ADDRH_BIT, OUTPUT);

  // vertical LED control pins
  pinMode(GPIO_SPI_NSS_BIT, OUTPUT);
  pinMode(GPIO_SPI_SCK_BIT, OUTPUT);
  pinMode(GPIO_SPI_MOSI_BIT, OUTPUT);
  pinMode(GPIO_N_MAT_ROE_BIT, OUTPUT);

  // buttons
  pinMode(GPIO_B1_BIT, INPUT_PULLUP);
  pinMode(GPIO_B2_BIT, INPUT_PULLUP);
  pinMode(GPIO_B3_BIT, INPUT_PULLUP);
  pinMode(GPIO_BOOT0_BIT, INPUT_PULLDOWN);

  // I2C bus
  pinMode(GPIO_SDA_BIT, OUTPUT_OPEN_DRAIN);
  pinMode(GPIO_SCL_BIT, OUTPUT_OPEN_DRAIN);
  pinMode(GPIO_N_ALM_BIT, INPUT_PULLUP);

  // phototransistor light sensor
  pinMode(GPIO_LIGHT_SENSE_BIT, INPUT);

  digitalWrite(GPIO_N_MAT_ROE_BIT, HIGH);
  digitalWrite(GPIO_N_MAT_ADDRL_BIT, HIGH);
  digitalWrite(GPIO_N_MAT_ADDRH_BIT, HIGH);
  digitalWrite(GPIO_SCL_BIT, HIGH);
  digitalWrite(GPIO_SCL_BIT, HIGH);
  digitalWrite(GPIO_SDA_BIT, HIGH);

  analogReference(AR_DEFAULT);

  // I2C comm parameters
  Wire.setSDA(GPIO_SDA_BIT);
  Wire.setSCL(GPIO_SCL_BIT);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeout(10);

  // enable test mode by pressing B1 and B3 together
  if (digitalRead(GPIO_B1_BIT) == LOW && digitalRead(GPIO_B3_BIT) == LOW) {
    init_status |= STATUS_TEST;
  }

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
      trimRTC(250);
      init_status |= STATUS_EBATTERY;
    }

    // clock output for measurement
    reg = bit(MCP79410_CONTROL_SQWEN);
    if (test_mode()) {
      reg |= bit(MCP79410_CONTROL_SQWFS0);
      reg |= bit(MCP79410_CONTROL_SQWFS1);
    }
    writeRTC(MCP79410_CONTROL, &reg, 1);

    // seed RNG
    uint8_t seed[4];
    readRTC(MCP79410_RTCSEC, seed, sizeof(seed));
    seedRand(*(int *)&seed[0]);

    // check if device was factory setup
    readEEPROM(0x00, seed, 4);
    if (*(uint32_t *)seed != 0xFAC7BABE) {
      *(uint32_t *)seed = 0xFAC7BABE;
      writeEEPROM(0x00, seed, 4);
      init_status |= STATUS_TEST;
    }

    // load last display
    readRTC(0x22, seed, 2);
    if ((seed[0] ^ seed[1]) == 0xFF) {
      if (seed[0] >= get_screens()) {
        seed[0] = 0;
      }
      display = seed[0];
      new_display = seed[0];
    }

    // load autotransition setting from RTC RAM
    readRTC(0x20, seed, 2);
    if ((seed[0] ^ seed[1]) == 0xFF) {
      autotransition_timeout = seed[0];
    }
  } else {
    init_status |= STATUS_ERTC;
  }

  // init clock calibration timer
  display_timer.setup(TIM14);
  display_timer.setPWM(1, NC, MAT_COLS * 100, 1, &display_begin, &display_end);
  display_timer.resume();

  // initialize SHT40
  if (!triggerSHT40()) {
    init_status |= STATUS_ESHT40;
  }

  // initialize light sensor value
  light_sensor = analogRead(GPIO_LIGHT_SENSE_BIT);

  // show error
  if (init_status != STATUS_OK) {
    if (init_status & STATUS_TEST) {
      display = 7;
      new_display = 7;
      autotransition_timeout = 0;
    }
    if (init_status & STATUS_EBATTERY) {
      strcat(str_buffer, " BAT! ");
    }
    if (init_status & STATUS_ERTC) {
      strcat(str_buffer, " RTC! ");
    }
    if (init_status & STATUS_ESHT40) {
      strcat(str_buffer, " SHT40! ");
    }
    showMessage(str_buffer, 128 * 4);
  }

  // do not allow to continue if buttons are stuck
  if (init_status & STATUS_TEST) {
    int test_state = 0;
    while (test_state < 9) {
      int frame = wait_frame();
      clearScreen(0);
      switch (test_state) {
        case 0:
          drawText(1, 1, "TEST!");
          if (frame >= 300) {
            test_state = 1;
          }
          break;
        case 1:
          drawText(1, 1, "PRE B1");
          if (digitalRead(GPIO_B1_BIT) == LOW) {
            test_state = 2;
          }
          break;
        case 2:
          drawText(1, 1, "REL B1");
          if (digitalRead(GPIO_B1_BIT) == HIGH) {
            test_state = 3;
          }
          break;
        case 3:
          drawText(1, 1, "PRE B2");
          if (digitalRead(GPIO_B2_BIT) == LOW) {
            test_state = 4;
          }
          break;
        case 4:
          drawText(1, 1, "REL B2");
          if (digitalRead(GPIO_B2_BIT) == HIGH) {
            test_state = 5;
          }
          break;
        case 5:
          drawText(1, 1, "PRE B3");
          if (digitalRead(GPIO_B3_BIT) == LOW) {
            test_state = 6;
          }
          break;
        case 6:
          drawText(1, 1, "REL B3");
          if (digitalRead(GPIO_B3_BIT) == HIGH) {
            test_state = 7;
          }
          break;
        case 7:
          drawText(1, 1, "P BOOT0");
          if (digitalRead(GPIO_BOOT0_BIT) == HIGH) {
            test_state = 8;
          }
          break;
        case 8:
          drawText(1, 1, "R BOOT0");
          if (digitalRead(GPIO_BOOT0_BIT) == LOW) {
            test_state = 9;
          }
          break;
      }
      swapBuffers();
    }
  }

}

void drawClock(char *str, bool reset)
{
  static char lastClock[9];
  static bool changed[8];
  static uint16_t image[MAT_COLS];
  static int animFrame = -1;
  if (reset) {
    animFrame = -1;
    strcpy(lastClock, str);
    drawText(1, 1, str);
    return;
  }

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
        backbuffer[a] = rotl16(image[a], animShift + 1);
      } else {
        backbuffer[a] = image[a] >> 7;
      }
    }
    x += w;
  }

  if (animShift >= 8) {
    animFrame = -1;
  }
}

#define SCREENS_BASE 7
#define SCREENS_TEST 3

int get_screens()
{
  int screens = SCREENS_BASE;
  if (test_mode()) {
    screens += SCREENS_TEST;
  }
  return screens;
}

void writeRTC_auto(bool b)
{
  uint8_t dat[] = { b, ~b };
  writeRTC(0x20, dat, sizeof(dat));
}

void writeRTC_display(uint8_t b)
{
  uint8_t dat[] = { b, ~b };
  writeRTC(0x22, dat, sizeof(dat));
}

void handle_button(int button)
{
  bool longpress = button & 0x80000000;
  button &= 0xFFFF;

  uint8_t data;
  uint8_t data2[3];
  bool tm = test_mode();
  int max_display = get_screens();
  if (game_active) {
    if (button == GPIO_BOOT0_BIT) {
      game_active = 0;
    } else {
      handle_button_game(button);
    }
    return;
  }
  if (button == GPIO_BOOT0_BIT) {
    if (longpress) {
      autotransition_timeout = !autotransition_timeout;
      writeRTC_auto(autotransition_timeout);
      if (autotransition_timeout) {
        showMessage("AUTO  on", 128 * 4);
      } else {
        showMessage("AUTO  off", 128 * 4);
      }
    } else if (msg_timer > 0) {
      msg_timer = 1;
    } else if (init_sequence == SEQ::IDLE) {
      if (++new_display >= max_display) {
        new_display = 0;
      }
      writeRTC_display(new_display);
    }
  }
  if (tm) {
    switch (button) {
      case GPIO_B3_BIT:
        --clock_trim;
        writeRTC_trim(clock_trim);
        break;
      case GPIO_B2_BIT:
        readRTC(MCP79410_CONTROL, &data, 1);
        data ^= bit(MCP79410_CONTROL_CRSTRIM);
        writeRTC(MCP79410_CONTROL, &data, 1);
        if (data & bit(MCP79410_CONTROL_CRSTRIM)) {
          showMessage("TRIM  on", 128*4);
        } else {
          showMessage("TRIM  off", 128*4);
        }
        break;
      case GPIO_B1_BIT:
        ++clock_trim;
        writeRTC_trim(clock_trim);
        break;
    }
  } else if (display == 0) {
    switch (button) {
      case GPIO_B3_BIT:
        readRTC(MCP79410_RTCSEC, &data, 1);
        data = (data & 0x80);
        writeRTC(MCP79410_RTCSEC, &data, 1);
        break;
      case GPIO_B2_BIT:
        readRTC(MCP79410_RTCMIN, &data, 1);
        data = bcdUnpack((bcdPack(data) + 1) % 60);
        writeRTC(MCP79410_RTCMIN, &data, 1);
        break;
      case GPIO_B1_BIT:
        readRTC(MCP79410_RTCHOUR, &data, 1);
        data = (data & 0xC0) | bcdUnpack((bcdPack(data & 0x3F) + 1) % 24);
        writeRTC(MCP79410_RTCHOUR, &data, 1);
        break;
    }
  } else if (display == 2) {
    switch (button) {
      case GPIO_B1_BIT:
        readRTC(MCP79410_RTCDATE, data2, 3);
        data = bcdPack(data2[0]) - 1;
        data = (data + 1) % getMonthDays(bcdPack(data2[1] & 0x1F), bcdPack(data2[2]));
        data = bcdUnpack(data + 1);
        writeRTC(MCP79410_RTCDATE, &data, 1);
        break;
      case GPIO_B2_BIT:
        readRTC(MCP79410_RTCMTH, &data, 1);
        data = bcdPack(data & 0x1F) - 1;
        data = (data + 1) % 12;
        data = bcdUnpack(data + 1);
        writeRTC(MCP79410_RTCMTH, &data, 1);
        break;
      case GPIO_B3_BIT:
        readRTC(MCP79410_RTCYEAR, &data, 1);
        data = bcdUnpack((bcdPack(data) + 1));
        writeRTC(MCP79410_RTCYEAR, &data, 1);
        break;
    }
  } else if (display == 6) {
    switch (button) {
      case GPIO_B3_BIT:
        game_active = 3;
        break;
      case GPIO_B2_BIT:
        game_active = 2;
        break;
      case GPIO_B1_BIT:
        game_active = 1;
        break;
    }
  }
}

bool update_buttons()
{
  bool button_pressed = false;
  for (int i = 0; i < countof(btn_map); i++) {
    if (digitalRead(btn_map[i]) == btn_map_state[i]) {
      if (btn_press[i] == 0) {
        int b = btn_map[i];
        if (btn_longpress[i]) {
          b |= 0x80000000;
        }
        handle_button(b);
        button_pressed = true;
      }
      if (++btn_press[i] >= 64) {
        btn_press[i] = 0;
        btn_longpress[i] = true;
      }
    } else {
      btn_press[i] = 0;
      btn_longpress[i] = false;
    }
  }
  if (button_pressed && autotransition_timeout) {
    autotransition_timeout = 1;
  }
  return button_pressed;
}

uint64_t wait_frame()
{
  static uint64_t last_frame_cnt;
  while (1) {
    uint64_t cur_frame_cnt = frame_cnt;
    if (cur_frame_cnt == last_frame_cnt) {
      asm("wfi");
    } else {
      last_frame_cnt = frame_cnt;
      return frame_cnt;
    }
  }
}

void loop() {
  // wait for new frame
  uint64_t frame_cnt = wait_frame();

  // load temperature & humidity each 32 frames
  if ((frame_cnt & 0x1F) == 0) {
      if (!readTemperatureHumidity(str_temp, str_rh)) {
        strcpy(str_temp, "SHT40?");
        strcpy(str_rh, "SHT40?");
      }
  }

  if (autotransition_timeout) {
    if (++autotransition_timeout >= 1024) {
      autotransition_timeout = 1;
      handle_button(GPIO_BOOT0_BIT);
    }
  }

  if (init_sequence == SEQ::WARMUP) {
    if (frame_cnt > 16) {
      // after 16 frames show intro animation
      init_sequence = SEQ::INTRO_S0;
    }
    // dont do anything else when in warm-up state
    return;
  }

  // process button inputs
  bool button_pressed = update_buttons();

  // redraw screen each 2nd frame or button was pressed
  if ((frame_cnt & 0x1) && !button_pressed) {
    return;
  }

  // here redraw screen in memory
  // execute this code every 2 frames
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
    swapBuffers();
    return;
  }

  if (msg_timer && msg_text) {
    static int msg_hold = 100;
    static int msg_shift;
    viewport_x = msg_shift >> 1;
    if (--msg_timer == 0) {
      msg_hold = 100;
      msg_shift = 0;
      viewport_x = 0;
      clearScreen(0);
      swapBuffers();
      clearScreen(0);
    } else {
      drawText(1, 1, msg_text);
      if (msg_hold <= 0) {
        ++msg_shift;
      } else {
        --msg_hold;
      }
    }
    swapBuffers();
    return;
  }

  switch (display) {
    case 7:
      str_buffer[0] = 'L';
      str_buffer[1] = ':';
      itoa(light_sensor, str_buffer+2, 10);
      strupr(str_buffer);
      break;
    case 8:
      str_buffer[0] = 'P';
      str_buffer[1] = ':';
      itoa(led_power, str_buffer+2, 10);
      break;
    case 9:
      clock_trim = readRTC_trim();
      str_buffer[0] = 'T';
      str_buffer[1] = ':';
      itoa(clock_trim, str_buffer+2, 10);
      break;
    default:
      break;
  }

  // update RTC only each 32 frames
  if ((frame_cnt & 0x1F) == 0 || button_pressed || clock_reload) {
    if (!readClock(display == 0 ? str_buffer : str_rh, display)) {
      strcpy(str_buffer, "RTC?");
    }
  }

  int textWidth = 0;
  switch(display) {
    case 0:
      drawClock(str_buffer, clock_reload);
      clock_reload = false;
      break;
    case 1:
      textWidth = drawText(-text_scroll + 1, 1, str_rh);
      break;
    case 2:
      textWidth = drawText(1, 1, str_rh);
      break;
    case 3:
      textWidth = drawText(-text_scroll + 1, 1, str_rh);
      break;
    case 4:
      textWidth = drawText(1, 1, str_temp);
      break;
    case 5:
      textWidth = drawText(1, 1, str_rh);
      break;
    case 6:
      if (game_active) {
        if (!drawGame(frame_cnt)) {
          game_active = 0;
        }
      } else {
        drawText(1, 1, "GAME");
      }
      break;
    default:
      textWidth = drawText(1, 1, str_buffer);
      break;
  }

  // display transition effect
  static int transition_cnt;
  int transition_frame = transition_cnt;
  int textDiff = textWidth - MAT_COLS;
  switch (init_sequence) {
    case SEQ::IDLE:
      if (new_display != display) {
        init_sequence = SEQ::DISPT1;
        autoscroll = false;
      }
      if (textWidth >= MAT_COLS) {
        // autoscroll text that exceeds screen width
        int mask = new_display != display ? 0x01 : 0x07;
        if ((frame_cnt & mask) == 0) {
          if (autoscroll) {
            if (--text_scroll <= -6) {
              autoscroll = !autoscroll;
            }
          } else {
            if (++text_scroll >= textDiff + 5) {
              autoscroll = !autoscroll;
            }
          }
        }
      }
      break;
    case SEQ::DISPT2:
      viewport_x = transition_frame;
      ++transition_cnt;
      if ((transition_frame % BUFFER_SIZE) == 0) {
        new_display = display;
        transition_cnt = 0;
        init_sequence = SEQ::IDLE;
      }
      break;
    case SEQ::DISPT1:
      viewport_x = transition_frame;
      ++transition_cnt;
      if (transition_frame >= MAT_COLS) {
        xchg(display, new_display);
        init_sequence = SEQ::DISPT2;
        text_scroll = 0;
        if (display == 0) {
          clock_reload = true;
        }
      }
      break;
  }

  swapBuffers();
}
