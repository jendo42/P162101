// This file was generated by JM Systems pin-generator tool dev-2-gdbd056b
// Part: IC2
// Device: STM32F042KXTX
#pragma once

// 'VSS2' -> 'GND' in schematic as Power style pin

// 'VSS1' -> 'GND' in schematic as Power style pin

// 'VDDA' -> '+3V3' in schematic as Power style pin

// 'VDDIO2' -> '+3V3' in schematic as Power style pin

// 'NRST' -> '!RST' in schematic as Unknown style pin
// ACTIVE-LOW

// 'PB4' -> '!ALM' in schematic as Atmel style pin
#define GPIO_N_ALM GPIO_BOOT0_BIT, GPIO_BOOT0_PORT
#define GPIO_N_ALM_BIT PB4
#define GPIO_N_ALM_NBIT 4
#define GPIO_N_ALM_PORT PORTB
#define GPIO_N_ALM_DDR DDR(GPIO_N_ALM_PORT)
#define GPIO_N_ALM_PIN PIN(GPIO_N_ALM_PORT)

// 'PB8' -> 'BOOT0' in schematic as Atmel style pin
#define GPIO_BOOT0 GPIO_BOOT0_BIT, GPIO_BOOT0_PORT
#define GPIO_BOOT0_BIT PB8
#define GPIO_BOOT0_NBIT 8
#define GPIO_BOOT0_PORT PORTB
#define GPIO_BOOT0_DDR DDR(GPIO_BOOT0_PORT)
#define GPIO_BOOT0_PIN PIN(GPIO_BOOT0_PORT)

// 'PA13' -> 'SWDIO' in schematic as Atmel style pin
#define GPIO_SWDIO GPIO_SWDIO_BIT, GPIO_SWDIO_PORT
#define GPIO_SWDIO_BIT PA13
#define GPIO_SWDIO_NBIT 13
#define GPIO_SWDIO_PORT PORTA
#define GPIO_SWDIO_DDR DDR(GPIO_SWDIO_PORT)
#define GPIO_SWDIO_PIN PIN(GPIO_SWDIO_PORT)

// 'VDD' -> 'VDD' in schematic as Power style pin

// 'PA0' -> 'MAT_ADDR0' in schematic as Atmel style pin
#define GPIO_MAT_ADDR0 GPIO_MAT_ADDR0_BIT, GPIO_MAT_ADDR0_PORT
#define GPIO_MAT_ADDR0_BIT PA0
#define GPIO_MAT_ADDR0_NBIT 0
#define GPIO_MAT_ADDR0_PORT PORTA
#define GPIO_MAT_ADDR0_DDR DDR(GPIO_MAT_ADDR0_PORT)
#define GPIO_MAT_ADDR0_PIN PIN(GPIO_MAT_ADDR0_PORT)

// 'PA1' -> 'MAT_ADDR1' in schematic as Atmel style pin
#define GPIO_MAT_ADDR1 GPIO_MAT_ADDR1_BIT, GPIO_MAT_ADDR1_PORT
#define GPIO_MAT_ADDR1_BIT PA1
#define GPIO_MAT_ADDR1_NBIT 1
#define GPIO_MAT_ADDR1_PORT PORTA
#define GPIO_MAT_ADDR1_DDR DDR(GPIO_MAT_ADDR1_PORT)
#define GPIO_MAT_ADDR1_PIN PIN(GPIO_MAT_ADDR1_PORT)

// 'PA2' -> 'MAT_ADDR2' in schematic as Atmel style pin
#define GPIO_MAT_ADDR2 GPIO_MAT_ADDR2_BIT, GPIO_MAT_ADDR2_PORT
#define GPIO_MAT_ADDR2_BIT PA2
#define GPIO_MAT_ADDR2_NBIT 2
#define GPIO_MAT_ADDR2_PORT PORTA
#define GPIO_MAT_ADDR2_DDR DDR(GPIO_MAT_ADDR2_PORT)
#define GPIO_MAT_ADDR2_PIN PIN(GPIO_MAT_ADDR2_PORT)

// 'PA3' -> 'MAT_ADDR3' in schematic as Atmel style pin
#define GPIO_MAT_ADDR3 GPIO_MAT_ADDR3_BIT, GPIO_MAT_ADDR3_PORT
#define GPIO_MAT_ADDR3_BIT PA3
#define GPIO_MAT_ADDR3_NBIT 3
#define GPIO_MAT_ADDR3_PORT PORTA
#define GPIO_MAT_ADDR3_DDR DDR(GPIO_MAT_ADDR3_PORT)
#define GPIO_MAT_ADDR3_PIN PIN(GPIO_MAT_ADDR3_PORT)

// 'PA15' -> 'SPI_NSS' in schematic as Atmel style pin
#define GPIO_SPI_NSS GPIO_SPI_NSS_BIT, GPIO_SPI_NSS_PORT
#define GPIO_SPI_NSS_BIT PA15
#define GPIO_SPI_NSS_NBIT 15
#define GPIO_SPI_NSS_PORT PORTA
#define GPIO_SPI_NSS_DDR DDR(GPIO_SPI_NSS_PORT)
#define GPIO_SPI_NSS_PIN PIN(GPIO_SPI_NSS_PORT)

// 'PB3' -> 'SPI_SCK' in schematic as Atmel style pin
#define GPIO_SPI_SCK GPIO_SPI_SCK_BIT, GPIO_SPI_SCK_PORT
#define GPIO_SPI_SCK_BIT PB3
#define GPIO_SPI_SCK_NBIT 3
#define GPIO_SPI_SCK_PORT PORTB
#define GPIO_SPI_SCK_DDR DDR(GPIO_SPI_SCK_PORT)
#define GPIO_SPI_SCK_PIN PIN(GPIO_SPI_SCK_PORT)

// 'PB5' -> 'SPI_MOSI' in schematic as Atmel style pin
#define GPIO_SPI_MOSI GPIO_SPI_MOSI_BIT, GPIO_SPI_MOSI_PORT
#define GPIO_SPI_MOSI_BIT PB5
#define GPIO_SPI_MOSI_NBIT 5
#define GPIO_SPI_MOSI_PORT PORTB
#define GPIO_SPI_MOSI_DDR DDR(GPIO_SPI_MOSI_PORT)
#define GPIO_SPI_MOSI_PIN PIN(GPIO_SPI_MOSI_PORT)

// 'PA4' -> '!MAT_ADDRL' in schematic as Atmel style pin
// ACTIVE-LOW
#define GPIO_N_MAT_ADDRL GPIO_N_MAT_ADDRL_BIT, GPIO_N_MAT_ADDRL_PORT
#define GPIO_N_MAT_ADDRL_BIT PA4
#define GPIO_N_MAT_ADDRL_NBIT 4
#define GPIO_N_MAT_ADDRL_PORT PORTA
#define GPIO_N_MAT_ADDRL_DDR DDR(GPIO_N_MAT_ADDRL_PORT)
#define GPIO_N_MAT_ADDRL_PIN PIN(GPIO_N_MAT_ADDRL_PORT)

// 'PA5' -> '!MAT_ADDRH' in schematic as Atmel style pin
// ACTIVE-LOW
#define GPIO_N_MAT_ADDRH GPIO_N_MAT_ADDRH_BIT, GPIO_N_MAT_ADDRH_PORT
#define GPIO_N_MAT_ADDRH_BIT PA5
#define GPIO_N_MAT_ADDRH_NBIT 5
#define GPIO_N_MAT_ADDRH_PORT PORTA
#define GPIO_N_MAT_ADDRH_DDR DDR(GPIO_N_MAT_ADDRH_PORT)
#define GPIO_N_MAT_ADDRH_PIN PIN(GPIO_N_MAT_ADDRH_PORT)

// 'PA6' -> '!MAT_ROE' in schematic as Atmel style pin
// ACTIVE-LOW
#define GPIO_N_MAT_ROE GPIO_N_MAT_ROE_BIT, GPIO_N_MAT_ROE_PORT
#define GPIO_N_MAT_ROE_BIT PA6
#define GPIO_N_MAT_ROE_NBIT 6
#define GPIO_N_MAT_ROE_PORT PORTA
#define GPIO_N_MAT_ROE_DDR DDR(GPIO_N_MAT_ROE_PORT)
#define GPIO_N_MAT_ROE_PIN PIN(GPIO_N_MAT_ROE_PORT)

// 'PA14' -> 'SWCLK' in schematic as Atmel style pin
#define GPIO_SWCLK GPIO_SWCLK_BIT, GPIO_SWCLK_PORT
#define GPIO_SWCLK_BIT PA14
#define GPIO_SWCLK_NBIT 14
#define GPIO_SWCLK_PORT PORTA
#define GPIO_SWCLK_DDR DDR(GPIO_SWCLK_PORT)
#define GPIO_SWCLK_PIN PIN(GPIO_SWCLK_PORT)

// 'PA8' -> 'B1' in schematic as Atmel style pin
#define GPIO_B1 GPIO_B1_BIT, GPIO_B1_PORT
#define GPIO_B1_BIT PA8
#define GPIO_B1_NBIT 8
#define GPIO_B1_PORT PORTA
#define GPIO_B1_DDR DDR(GPIO_B1_PORT)
#define GPIO_B1_PIN PIN(GPIO_B1_PORT)

// 'PA9' -> 'B2' in schematic as Atmel style pin
#define GPIO_B2 GPIO_B2_BIT, GPIO_B2_PORT
#define GPIO_B2_BIT PA9
#define GPIO_B2_NBIT 9
#define GPIO_B2_PORT PORTA
#define GPIO_B2_DDR DDR(GPIO_B2_PORT)
#define GPIO_B2_PIN PIN(GPIO_B2_PORT)

// 'PA10' -> 'B3' in schematic as Atmel style pin
#define GPIO_B3 GPIO_B3_BIT, GPIO_B3_PORT
#define GPIO_B3_BIT PA10
#define GPIO_B3_NBIT 10
#define GPIO_B3_PORT PORTA
#define GPIO_B3_DDR DDR(GPIO_B3_PORT)
#define GPIO_B3_PIN PIN(GPIO_B3_PORT)

// 'PB1' -> 'LIGHT_SENSE' in schematic as Atmel style pin
#define GPIO_LIGHT_SENSE GPIO_LIGHT_SENSE_BIT, GPIO_LIGHT_SENSE_PORT
#define GPIO_LIGHT_SENSE_BIT PB1
#define GPIO_LIGHT_SENSE_NBIT 1
#define GPIO_LIGHT_SENSE_PORT PORTB
#define GPIO_LIGHT_SENSE_DDR DDR(GPIO_LIGHT_SENSE_PORT)
#define GPIO_LIGHT_SENSE_PIN PIN(GPIO_LIGHT_SENSE_PORT)

// 'PB6' -> 'SCL' in schematic as Atmel style pin
#define GPIO_SCL GPIO_SCL_BIT, GPIO_SCL_PORT
#define GPIO_SCL_BIT PB6
#define GPIO_SCL_NBIT 6
#define GPIO_SCL_PORT PORTB
#define GPIO_SCL_DDR DDR(GPIO_SCL_PORT)
#define GPIO_SCL_PIN PIN(GPIO_SCL_PORT)

// 'PB7' -> 'SDA' in schematic as Atmel style pin
#define GPIO_SDA GPIO_SDA_BIT, GPIO_SDA_PORT
#define GPIO_SDA_BIT PB7
#define GPIO_SDA_NBIT 7
#define GPIO_SDA_PORT PORTB
#define GPIO_SDA_DDR DDR(GPIO_SDA_PORT)
#define GPIO_SDA_PIN PIN(GPIO_SDA_PORT)

// other defs

#define MAT_COLS 29

#define MAT_COL_ADDR_MASK ( \
    bit(GPIO_MAT_ADDR0_NBIT) \
  | bit(GPIO_MAT_ADDR1_NBIT) \
  | bit(GPIO_MAT_ADDR2_NBIT) \
  | bit(GPIO_MAT_ADDR3_NBIT) \
)

#define MAT_COL_CTRL_MASK ( \
    bit(GPIO_N_MAT_ADDRL_NBIT) \
  | bit(GPIO_N_MAT_ADDRH_NBIT) \
)
