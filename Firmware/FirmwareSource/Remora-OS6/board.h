
#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "mbed.h"
#include "PeripheralPins.h"
#include "hal/gpio_api.h"

#ifdef SDIO_BASE
#define SD_BASE SDIO_BASE
#elif defined SDIO1_BASE
#define SD_BASE SDIO1_BASE
#elif defined SDMMC1_BASE
#define SD_BASE SDMMC1_BASE
#elif defined SDMMC2_BASE
#define SD_BASE SDMMC2_BASE
#elif defined SDMMC_BASE
#define SD_BASE SDMMC_BASE
#endif


void init_board_pins(void);
void init_sdio(void);
void deinit_sdio(void);
void init_system_clocks(void);
void set_sdio_clock(bool enable, PinName sdio_ck);
void init_spi1(void);
void init_spi2(void);
void init_spi3(void);
void init_probe_pins(void);
void init_ui_pins(void);
int init_sdio_interface(void);
void sdio_power_control(bool enable);
bool is_sd_card_present(void);
void set_sdio_clock(bool enable);

#endif
