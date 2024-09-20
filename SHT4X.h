#pragma once

#define SHT4X_ADDR 0x44 // I2C address
#define SHT4X_HPM 0xFD // high precision measure
#define SHT4X_LPM 0xE0 // low precision measur

#define STATUS_ESHT40     4

bool readSHT40(int *out_t, int *out_rh);
bool triggerSHT40();
