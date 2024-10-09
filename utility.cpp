#include "utility.h"

static int _seed;
static int rand_last;

static const uint8_t CRC_TABLE[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

uint8_t decode6to8(uint8_t code)
{
  switch (code) {
      case 0b01011001:
        return 0;
      case 0b01110001:
        return 1;
      case 0b01110010:
        return 2;
      case 0b11000011:
        return 3;
      case 0b01100101:
        return 4;
      case 0b11000101:
        return 5;
      case 0b11000110:
        return 6;
      case 0b10000111:
        return 7;
      case 0b01101001:
        return 8;
      case 0b11001001:
        return 9;
      case 0b11001010:
        return 10;
      case 0b10001011:
        return 11;
      case 0b11001100:
        return 12;
      case 0b10001101:
        return 13;
      case 0b10001110:
        return 14;
      case 0b01001011:
        return 15;
      case 0b01010011:
        return 16;
      case 0b11010001:
        return 17;
      case 0b11010010:
        return 18;
      case 0b10010011:
        return 19;
      case 0b11010100:
        return 20;
      case 0b10010101:
        return 21;
      case 0b10010110:
        return 22;
      case 0b00010111:
        return 23;
      case 0b11011000:
        return 24;
      case 0b10011001:
        return 25;
      case 0b10011010:
        return 26;
      case 0b00011011:
        return 27;
      case 0b10011100:
        return 28;
      case 0b00011101:
        return 29;
      case 0b00011110:
        return 30;
      case 0b01011100:
        return 31;
      case 0b01100011:
        return 32;
      case 0b11100001:
        return 33;
      case 0b11100010:
        return 34;
      case 0b10100011:
        return 35;
      case 0b11100100:
        return 36;
      case 0b10100101:
        return 37;
      case 0b10100110:
        return 38;
      case 0b00100111:
        return 39;
      case 0b11101000:
        return 40;
      case 0b10101001:
        return 41;
      case 0b10101010:
        return 42;
      case 0b00101011:
        return 43;
      case 0b10101100:
        return 44;
      case 0b00101101:
        return 45;
      case 0b00101110:
        return 46;
      case 0b01101100:
        return 47;
      case 0b01110100:
        return 48;
      case 0b10110001:
        return 49;
      case 0b10110010:
        return 50;
      case 0b00110011:
        return 51;
      case 0b10110100:
        return 52;
      case 0b00110101:
        return 53;
      case 0b00110110:
        return 54;
      case 0b01010110:
        return 55;
      case 0b10111000:
        return 56;
      case 0b00111001:
        return 57;
      case 0b00111010:
        return 58;
      case 0b01011010:
        return 59;
      case 0b00111100:
        return 60;
      case 0b01001101:
        return 61;
      case 0b01001110:
        return 62;
      case 0b01100110:
        return 63;
      default:
        return 0xFF;
  }
}

uint8_t crc8itu_update(uint8_t val, uint8_t data)
{
  return CRC_TABLE[val ^ data];
}

uint8_t crc8itu(const void * data, size_t size)
{
	uint8_t val = 0;

	uint8_t * pos = (uint8_t *) data;
	uint8_t * end = pos + size;

	while (pos < end) {
		val = crc8itu_update(val, *pos);
		pos++;
	}

	return val;
}

uint8_t checksum(const void * data, size_t size)
{
	uint8_t val = 0;

	uint8_t * pos = (uint8_t *) data;
	uint8_t * end = pos + size;

	while (pos < end) {
		val += *pos;
		pos++;
	}

	return -val;
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
  if (c >= 10) {
    c += 'A' - 10;
  } else {
    c += '0';
  }
  str[0] = c;
  str[1] = 0;
}

void byte2hex(uint8_t value, char *str)
{
  nibble2hex(value >> 4, str);
  nibble2hex(value, str + 1);
}

void word2hex(uint16_t value, char *str)
{
  byte2hex(value >> 8, str);
  byte2hex(value, str + 2);
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

int getMonthDays(uint8_t month, uint8_t year)
{
  static const uint8_t month_days[] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
  };

  if (year > 99) {
    return 0;
  }

  int y = 2000 + year;
  int m = (month - 1) % 12;
  int d = month_days[m];
  if ((y & 3) == 0 && m == 1) {
    // leap year
    ++d;
  }

  return d;
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
