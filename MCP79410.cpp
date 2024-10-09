
#include <Wire.h>
#include "MCP79410.h"
#include "utility.h"

extern int init_status;

size_t readRTC(uint8_t address, uint8_t *buffer, size_t size)
{
  if (init_status & STATUS_ERTC) {
    return 0;
  }
  Wire.beginTransmission(MCP79410_ADDR_RTC);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(MCP79410_ADDR_RTC, size);
  return Wire.readBytes(buffer, size);
}

uint8_t writeRTC(uint8_t address, uint8_t *buffer, size_t size)
{
  if (init_status & STATUS_ERTC) {
    return 0;
  }
  Wire.beginTransmission(MCP79410_ADDR_RTC);
  Wire.write(address);
  Wire.write(buffer, size);
  return Wire.endTransmission();
}

int readRTC_trim()
{
  uint8_t data;
  readRTC(MCP79410_OSCTRIM, &data, 1);
  int trim = data & 0x7F;
  if (~data & 0x80) {
    trim = -trim;
  }
  return trim;
}

void writeRTC_trim(int trim)
{
  uint8_t data;
  if (trim < 0) {
    data = -trim & 0x7F;
  } else {
    data = (trim & 0x7F) | 0x80;
  }
  writeRTC(MCP79410_OSCTRIM, &data, 1);
}

void trimRTC(int temp)
{
  // measured with 10p caps @ 26.5°C
  // compensated to 25°C
  const int base_error = 26;
  int temp_error = clock_error(temp); 
  writeRTC_trim(base_error - temp_error);
}

size_t readEEPROM(uint8_t address, uint8_t *buffer, size_t size)
{
  if (init_status & STATUS_ERTC) {
    return 0;
  }
  Wire.beginTransmission(MCP79410_ADDR_EEPROM);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(MCP79410_ADDR_EEPROM, size);
  return Wire.readBytes(buffer, size);
}

// writes only 1 byte or whole page (8 bytes)
uint8_t writeEEPROM(uint8_t address, uint8_t *buffer, size_t size)
{
  if (init_status & STATUS_ERTC) {
    return 0;
  }
  Wire.beginTransmission(MCP79410_ADDR_EEPROM);
  Wire.write(address);
  Wire.write(buffer, size);
  return Wire.endTransmission();
}
