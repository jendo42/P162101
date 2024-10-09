#include "Arduino.h"

#include "light_receiver.h"
#include "utility.h"
#include "MCP79410.h"
#include "P162101.h"

static bool light_sync = false;
static int light_symbol_state = 0;
static uint32_t light_symbol_frame;
static uint8_t light_symbol_data;
static int light_symbol_noise = 0;
static int tick = 0;

int light_receiver_status()
{
  return tick;
}

void light_symbol_push_one()
{
  light_symbol_frame |= 0x400;
  light_symbol_frame >>= 1;
  ++light_symbol_state;
}

void light_symbol_push_zero()
{
  light_symbol_frame >>= 1;
  ++light_symbol_state;
}

bool light_symbol()
{
  int light = ((int)analogRead(GPIO_LIGHT_SENSE_BIT) - light_symbol_noise) >> 4;
  return light <= 0;
}

void update_clock(uint8_t data, uint8_t index, uint8_t seq)
{
  const static uint8_t index2reg[] = {
    MCP79410_RTCHOUR,
    MCP79410_RTCMIN,
    MCP79410_RTCSEC,
    MCP79410_RTCDATE,
    MCP79410_RTCMTH,
    MCP79410_RTCYEAR
  };
  const static uint8_t rtcMask[] = {
    0x7F,
    0x7F,
    0x3F,
    0x07,
    0x3F,
    0x1F,
    0xFF
  };
  static uint8_t buf[7];
  static uint8_t lastSeq;

  if (lastSeq != seq) {
    if (index == 0) {
      lastSeq = seq;
    } else {
      return;
    }
  }

  index %= sizeof(index2reg);
  buf[index2reg[index]] = data;

  if (index == sizeof(index2reg) - 1) {
    // write data to RTC
    uint8_t buf2[7];
    readRTC(MCP79410_RTCSEC, buf2, sizeof(buf2));
    for (int i = 0; i < sizeof(rtcMask); i++) {
      buf2[i] = (buf2[i] & ~rtcMask[i]) | (buf[i] & rtcMask[i]);
    }
    writeRTC(MCP79410_RTCSEC, buf2, sizeof(buf2));
    tick = seq;
  }
}

void decode_light_frame(uint8_t data)
{
  static int frame_pos = 0;
  static uint8_t data_buf[4];
  static bool frame_sync = false;
  if (!frame_sync) {
    if (data == 0x55) {
      frame_sync = true;
    }
    return;
  }

  data_buf[frame_pos++] = data;
  if (frame_pos >= sizeof(data_buf)) {
    frame_sync = false;
    frame_pos = 0;
    for (int i = 0; i < sizeof(data_buf); i++) {
      data_buf[i] = decode6to8(data_buf[i]);
    }

    data_buf[0] |= (data_buf[3] & (3 << 0)) << 6;
    data_buf[1] |= (data_buf[3] & (3 << 2)) << 4;
    data_buf[2] |= (data_buf[3] & (3 << 4)) << 2;

    if (crc8itu(data_buf, 3) == 0) {
      // data is valid!
      //x = *(uint16_t *)data_buf;
      update_clock(data_buf[1], data_buf[0] & 0x0F, data_buf[0] >> 4);
    }
  }
}

void light_receiver(int light_sensor)
{
  static int bsc = 0;
  static bool last_light_symbol = false;
  bool current_light_symbol = light_symbol();

  light_symbol_noise += (light_sensor - light_symbol_noise) >> 1;

  switch (light_symbol_state) {
    case 0:
      if (current_light_symbol == false && last_light_symbol == true) {
        // start bit
        bsc = 1;
        light_symbol_state = 1;
      }
      break;
    default:
      if (last_light_symbol != current_light_symbol) {
        // transition, reset sample counter
        bsc = 0;
      }
      if (++bsc >= 9) {
        bsc = 0;
        if (current_light_symbol) {
          light_symbol_push_one();
        } else {
          light_symbol_push_zero();
        }
      }
      if (light_symbol_state >= 11) {
        if ((light_symbol_frame & 0x201) == 0x200) {
          light_symbol_data = light_symbol_frame >> 1;
          decode_light_frame(light_symbol_data);
        }
        light_symbol_state = 0;
      }
      break;
  }

  last_light_symbol = current_light_symbol;
}
