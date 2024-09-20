#include <Wire.h>
#include "SHT4X.h"

extern int init_status;

static int32_t sht40_lastCommandTime = -1;
bool triggerSHT40()
{
  if (sht40_lastCommandTime != -1 || (init_status & STATUS_ESHT40)) {
    return false;
  }

  Wire.beginTransmission(SHT4X_ADDR);
  Wire.write(SHT4X_LPM);
  if (Wire.endTransmission() == 0) {
    sht40_lastCommandTime = millis();
    return true;
  }

  return false;
}

// temperature is in deg. Celsius * 10
// (one decimal place fixed point)
bool readSHT40(int *out_t, int *out_rh)
{
  if (sht40_lastCommandTime == -1 || (init_status & STATUS_ESHT40)) {
    return false;
  }

  uint8_t buff[6];
  uint32_t elapsed = millis() - sht40_lastCommandTime;
  if (elapsed > 10) {
    // read data from sensor
    sht40_lastCommandTime = -1;
    if (Wire.requestFrom(SHT4X_ADDR, sizeof(buff)) != sizeof(buff)) {
      return false;
    }
    Wire.readBytes(buff, sizeof(buff));
    if (out_t) {
      int val = (buff[0] << 8) | buff[1];
      *out_t = -450 + 1750 * val / 65535;
    }
    if (out_rh) {
      int val = (buff[3] << 8) | buff[4];
      int lrh = -60 + 1250 * val / 65535;
      if (lrh > 1000) {
        lrh = 1000;
      }
      if (lrh < 0) {
        lrh = 0;
      }
      *out_rh = lrh;
    }
  }

  return true;
}
