#pragma once
#include <limits.h>
#include <stdint.h>

#define countof(x) (sizeof(x) / sizeof(*x))
#define LEAP_YEAR(Y)     ( (Y>0) && !(Y%4) && ( (Y%100) || !(Y%400) ))     // from time-lib

static inline void xchg(int &a,  int &b)
{
  int x = a;
  a = b;
  b = x;
}

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

static inline uint32_t rotr32(uint32_t n, unsigned int c)
{
  const unsigned int mask = (CHAR_BIT*sizeof(n) - 1);
  c &= mask;
  return (n>>c) | (n<<( (-c)&mask )); 
}

uint8_t crc8itu_update(uint8_t val, uint8_t data);
uint8_t crc8itu(const void * data, size_t size);
uint8_t checksum(const void * data, size_t size);
uint8_t decode6to8(uint8_t code);

uint8_t bcdPack(uint8_t x);
uint8_t bcdUnpack(uint8_t x);
void nibble2hex(uint8_t value, char *str);
void byte2hex(uint8_t value, char *str);
void word2hex(uint16_t value, char *str);

// WARNING: this does not terminate string with '\0'
void value2str(int value, char *str);

// calculates clock error in PPM from actual ambient temperature
int clock_error(int T);

// calculates precise day of week
int dayOfWeek(uint8_t year, uint8_t month, uint8_t day);
int getMonthDays(uint8_t month, uint8_t year);

// lightweight linear pseudo-RNG
int getRand();
void seedRand(int seed);
