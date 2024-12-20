#pragma once

#define MCP79410_ADDR_RTC 0x6F
#define MCP79410_ADDR_EEPROM 0x57

 #define MCP79410_RTCSEC 0x00
 #define MCP79410_RTCMIN 0x01
 #define MCP79410_RTCHOUR 0x02
 #define MCP79410_RTCWKDAY 0x03
 #define MCP79410_RTCDATE 0x04
 #define MCP79410_RTCMTH 0x05
 #define MCP79410_RTCYEAR 0x06
 #define MCP79410_CONTROL 0x07
 #define MCP79410_OSCTRIM 0x08
 #define MCP79410_EEUNLOCK 0x09

#define MCP79410_RTCWKDAY_OSCRUN 5
#define MCP79410_RTCWKDAY_PWRFAIL 4
#define MCP79410_RTCWKDAY_VBATEN 3
#define MCP79410_RTCWKDAY_WKDAY2 2
#define MCP79410_RTCWKDAY_WKDAY1 1
#define MCP79410_RTCWKDAY_WKDAY0 0

#define MCP79410_CONTROL_SQWEN 6
#define MCP79410_CONTROL_CRSTRIM 2
#define MCP79410_CONTROL_SQWFS1 1
#define MCP79410_CONTROL_SQWFS0 0

#define STATUS_EBATTERY   1
#define STATUS_ERTC       2

size_t readRTC(uint8_t address, uint8_t *buffer, size_t size);
uint8_t writeRTC(uint8_t address, uint8_t *buffer, size_t size);
int readRTC_trim();
void writeRTC_trim(int trim);
void trimRTC(int temp);
size_t readEEPROM(uint8_t address, uint8_t *buffer, size_t size);
// writes only 1 byte or whole page (8 bytes)
uint8_t writeEEPROM(uint8_t address, uint8_t *buffer, size_t size);
