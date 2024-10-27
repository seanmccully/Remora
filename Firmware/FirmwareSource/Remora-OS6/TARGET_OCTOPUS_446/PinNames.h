/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2018, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"
#include "PinNamesTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ALT0  = 0x100,
    ALT1  = 0x200,
    ALT2  = 0x300,
    ALT3  = 0x400
} ALTx;

typedef enum {
    // STM32 Pin Names
    PA_0  = 0x00,  // DRIVER3_EN
    PA_1  = 0x01,  // HEATER_BED
    PA_2  = 0x02,  // HEATER_0
    PA_3  = 0x03,  // HEATER_1
    PA_4  = 0x04,  // SD_CS
    PA_5  = 0x05,  // SPI1_SCK
    PA_6  = 0x06,  // SPI1_MISO
    PA_7  = 0x07,  // SPI1_MOSI
    PA_8  = 0x08,  // FAN0
    PA_9  = 0x09,  // UART1_TX
    PA_10 = 0x0A,  // UART1_RX
    PA_11 = 0x0B,  // USB_DM
    PA_12 = 0x0C,  // USB_DP
    PA_13 = 0x0D,  // SWDIO/WORK_LED
    PA_14 = 0x0E,  // DRIVER7_DIR
    PA_14_ALT = 0x0E,  // SWCLK
    PA_15 = 0x0F,  // SPI3_NSS
    PA_15_ALT = 0x0F,  // JTDI

    PB_0  = 0x10,  // STATUS_LED
    PB_1  = 0x11,  // BTN_EN2
    PB_2  = 0x12,  // BTN_EN1
    PB_3  = 0x13,  // SPI3_SCK
    PB_3_ALT  = 0x13,  // JTDO SWO
    PB_4  = 0x14,  // SPI3_MISO
    PB_4_ALT  = 0x14,  // JTRST
    PB_5  = 0x15,  // SPI3_MOSI
    PB_6  = 0x16,  // PROBE_PWM
    PB_7  = 0x17,  // PROBE_SENSE
    PB_8  = 0x18,  // I2C1_SCL
    PB_9  = 0x19,  // I2C1_SDA
    PB_10 = 0x1A,  // HEATER_2
    PB_11 = 0x1B,  // HEATER_3
    PB_12 = 0x1C,  // SPI2_NSS
    PB_13 = 0x1D,  // SPI2_SCK
    PB_14 = 0x1E,  // USB_HS_DM
    PB_15 = 0x1F,  // USB_HS_DP

    PC_0  = 0x20,  // PWR_DET
    PC_1  = 0x21,  // DRIVER3_DIR
    PC_2  = 0x22,  // SPI2_MISO
    PC_3  = 0x23,  // SPI2_MOSI
    PC_4  = 0x24,  // DRIVER0_CS
    PC_5  = 0x25,  // PROBE_IN
    PC_6  = 0x26,  // DRIVER2_CS
    PC_7  = 0x27,  // DRIVER3_CS
    PC_8  = 0x28,  // SDIO_D0
    PC_9  = 0x29,  // SDIO_D1
    PC_10 = 0x2A,  // SDIO_D2
    PC_11 = 0x2B,  // SDIO_D3
    PC_12 = 0x2C,  // SDIO_CK
    PC_13 = 0x2D,  // DRIVER5_STEP
    PC_14 = 0x2E,  // OSC32_IN
    PC_15 = 0x2F,  // OSC32_OUT

    PD_0  = 0x30,  // CAN1_RX
    PD_1  = 0x31,  // CAN1_TX
    PD_2  = 0x32,  // SDIO_CMD
    PD_3  = 0x33,  // DRIVER7_CS
    PD_4  = 0x34,  // DRIVER6_EN
    PD_5  = 0x35,  // UART2_TX
    PD_6  = 0x36,  // UART2_RX
    PD_7  = 0x37,  // ESP_TX
    PD_8  = 0x38,  // UART3_TX
    PD_9  = 0x39,  // UART3_RX
    PD_10 = 0x3A,  // ESP_RX
    PD_11 = 0x3B,  // DRIVER1_CS
    PD_12 = 0x3C,  // FAN2
    PD_13 = 0x3D,  // FAN3
    PD_14 = 0x3E,  // FAN4
    PD_15 = 0x3F,  // FAN5

    PE_0  = 0x40,  // DRIVER7_EN
    PE_1  = 0x41,  // DRIVER6_CS
    PE_2  = 0x42,  // DRIVER6_STEP
    PE_3  = 0x43,  // DRIVER6_DIR
    PE_4  = 0x44,  // DRIVER5_CS
    PE_5  = 0x45,  // FAN1
    PE_6  = 0x46,  // DRIVER7_STEP
    PE_7  = 0x47,  // BTN_ENC
    PE_8  = 0x48,  // BEEPER
    PE_9  = 0x49,  // LCD_EN
    PE_10 = 0x4A,  // LCD_RS
    PE_11 = 0x4B,  // PS_ON
    PE_12 = 0x4C,  // LCD_D4
    PE_13 = 0x4D,  // LCD_D5
    PE_14 = 0x4E,  // LCD_D6
    PE_15 = 0x4F,  // LCD_D7

    PF_0  = 0x50,  // DRIVER5_DIR
    PF_1  = 0x51,  // DRIVER5_EN
    PF_2  = 0x52,  // DRIVER4_CS
    PF_3  = 0x53,  // THERM0
    PF_4  = 0x54,  // THERM1
    PF_5  = 0x55,  // THERM2
    PF_6  = 0x56,  // THERM3
    PF_7  = 0x57,  // THERM4
    PF_8  = 0x58,  // MAX31865_CS
    PF_9  = 0x59,  // DRIVER4_STEP
    PF_10 = 0x5A,  // DRIVER4_DIR
    PF_11 = 0x5B,  // DRIVER2_STEP
    PF_12 = 0x5C,  // DRIVER0_DIR
    PF_13 = 0x5D,  // DRIVER0_STEP
    PF_14 = 0x5E,  // DRIVER0_EN
    PF_15 = 0x5F,  // DRIVER1_EN

    PG_0  = 0x60,  // DRIVER1_STEP
    PG_1  = 0x61,  // DRIVER1_DIR
    PG_2  = 0x62,  // DRIVER4_EN
    PG_3  = 0x63,  // DRIVER2_DIR
    PG_4  = 0x64,  // DRIVER3_STEP
    PG_5  = 0x65,  // DRIVER2_EN
    PG_6  = 0x66,  // X_MIN
    PG_7  = 0x67,  // ESP_IO0
    PG_8  = 0x68,  // ESP_RESET
    PG_9  = 0x69,  // X_MAX
    PG_10 = 0x6A,  // Y_MIN
    PG_11 = 0x6B,  // Y_MAX
    PG_12 = 0x6C,  // Z_MIN
    PG_13 = 0x6D,  // Z_MAX
    PG_14 = 0x6E,  // E_MIN
    PG_15 = 0x6F,  // E_MAX

    // ADC internal channels
    ADC_TEMP = 0xF0,
    ADC_VREF = 0xF1,
    ADC_VBAT = 0xF2,
//
    // Arduino connector namings
    A0          = PA_3,
    A1          = PC_0,
    A2          = PC_3,
    A3          = PF_3,
    A4          = PF_5,
    A5          = PF_10,
    D0          = PG_9,
    D1          = PG_14,
    D2          = PF_15,
    D3          = PE_13,
    D4          = PF_14,
    D5          = PE_11,
    D6          = PE_9,
    D7          = PF_13,
    D8          = PF_12,
    D9          = PD_15,
    D10         = PD_14,
    D11         = PA_7,
    D12         = PA_6,
    D13         = PA_5,
    D14         = PB_9,
    D15         = PB_8,


    // STDIO for console print
#ifdef MBED_CONF_TARGET_STDIO_UART_TX
    STDIO_UART_TX = MBED_CONF_TARGET_STDIO_UART_TX,
#else
    STDIO_UART_TX = PD_8,
#endif
#ifdef MBED_CONF_TARGET_STDIO_UART_RX
    STDIO_UART_RX = MBED_CONF_TARGET_STDIO_UART_RX,
#else
    STDIO_UART_RX = PD_9,
#endif

    CONSOLE_TX   = STDIO_UART_TX, // Virtual Com Port
    CONSOLE_RX   = STDIO_UART_RX, // Virtual Com Port
    //USBTX       = STDIO_UART_TX, // Virtual Com Port
    //USBRX       = STDIO_UART_RX, // Virtual Com Port
    I2C_SCL     = D15,
    I2C_SDA     = D14,
    SPI_MOSI    = D11,
    SPI_MISO    = D12,
    SPI_SCK     = D13,
    SPI_CS      = D10,
    PWM_OUT     = D9,


    // Not connected
    NC = (int)0xFFFFFFFF
} PinName;

#ifdef __cplusplus
}
#endif

#endif
