#include "utility.h"

static int _seed;
static int rand_last;

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

int clock_error(int T)
{
  const int T0 = 250; // Turnover temperature
  const int k = -34; // Temperature coeficient
  int64_t dT = T - T0;
  dT *= dT;
  dT *= k;
  return dT / 100000;
}

// https://forum.arduino.cc/t/arduino-clock/192225/4
int dayOfWeek(uint8_t y, uint8_t m, uint8_t d)
{
  // Old mental arithmetic method for calculating day of week
  // adapted for Arduino, for years 2000~2099
  // returns 1 for Sunday, 2 for Monday, etc., up to 7 for Saturday
  // for "bad" dates (like Feb. 30), it returns 0
  // Note: input year (y) should be a number from 0~99
  if (y > 99) return 0; // we don't accept years after 2099
  // we take care of bad months later
  if (d < 1) return 0; // because there is no day 0
  int w = 6; // this is a magic number (y2k fix for this method)
  // one ordinary year is 52 weeks + 1 day left over
  // a leap year has one more day than that
  // we add in these "leftover" days
  w += (y + (y >> 2));
  // correction for Jan. and Feb. of leap year
  if (((y & 3) == 0) && (m <= 2)) w--;
  // add in "magic number" for month
  switch (m) {
    case 1:  if (d > 31) return 0; w += 1; break;
    case 2:  if (d > ((y & 3) ? 28 : 29)) return 0; w += 4; break;
    case 3:  if (d > 31) return 0; w += 4; break;
    case 4:  if (d > 30) return 0; break;
    case 5:  if (d > 31) return 0; w += 2; break;
    case 6:  if (d > 30) return 0; w += 5; break;
    case 7:  if (d > 31) return 0; break;
    case 8:  if (d > 31) return 0; w += 3; break;
    case 9:  if (d > 30) return 0; w += 6; break;
    case 10: if (d > 31) return 0; w += 1; break;
    case 11: if (d > 30) return 0; w += 4; break;
    case 12: if (d > 31) return 0; w += 6; break;
    default: return 0;
  }
  // then add day of month
  w += d;
  // there are only 7 days in a week, so we "cast out" sevens
  while (w > 7) w = (w >> 3) + (w & 7);
  return w;
}

int getRand()
{
  int val = (rand_last * 1103515245) + 12345;
  rand_last = val;
  return (val ^ _seed) & 0x7fffffff;
}

void seedRand(int seed)
{
  _seed = seed;
  rand_last ^= seed;
}
