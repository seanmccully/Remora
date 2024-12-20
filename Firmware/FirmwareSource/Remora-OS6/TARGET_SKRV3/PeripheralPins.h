/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2014, STMicroelectronics
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

#ifndef MBED_PERIPHERALPINS_H
#define MBED_PERIPHERALPINS_H

#include "pinmap.h"
#include "PeripheralNames.h"
#include "mbed_toolchain.h"

#ifndef USB_OTG1_PERIPH_BASE
#define USB_OTG1_PERIPH_BASE       0x40080000UL
#endif

#ifndef USB_OTG2_PERIPH_BASE  
#define USB_OTG2_PERIPH_BASE       0x40040000UL
#endif
// Then define the USB OTG FS base using OTG1
#define USB_OTG_FS      USB_OTG1_PERIPH_BASE

#ifndef RCC_BASE
#define RCC_BASE              0x58024400UL
#endif

// Define RCC oscillator types - these should match RCC HAL definitions
#define RCC_OSC_IN           RCC_BASE
#define RCC_OSC_OUT          RCC_BASE

// Debug Interface Types
typedef enum {
    DEBUG_JTMS_SWDIO = 0x0,  // JTAG TMS / SWD IO
    DEBUG_JTCK_SWCLK = 0x1,  // JTAG TCK / SWD CLK
    DEBUG_JTDI       = 0x2,  // JTAG TDI
    DEBUG_JTDO_SWO   = 0x3,  // JTAG TDO / SWO
    DEBUG_JTRST      = 0x4,  // JTAG TRST
    DEBUG_SWD        = 0x5,  // SWD (Combined SWDIO/SWCLK)
    DEBUG_ETM        = 0x6   // Embedded Trace
} DebugName;

extern const PinMap PinMap_ADC[];
extern const PinMap PinMap_ADC_Internal[];
extern const PinMap PinMap_DAC[];
extern const PinMap PinMap_UART_RTS[];
extern const PinMap PinMap_UART_CTS[];
extern const PinMap PinMap_PWM[];
extern const PinMap PinMap_SPI_MOSI[];
extern const PinMap PinMap_SPI_MISO[];
extern const PinMap PinMap_SPI_SCLK[];
extern const PinMap PinMap_SPI_SSEL[];
extern const PinMap PinMap_STEPPER_STEP[];
extern const PinMap PinMap_STEPPER_DIR[];
extern const PinMap PinMap_STEPPER_ENABLE[];
extern const PinMap PinMap_ENDSTOPS[];
extern const PinMap PinMap_PROBE[];
extern const PinMap PinMap_HEATER[];
extern const PinMap PinMap_FAN[];
extern const PinMap PinMap_TEMP[];
extern const PinMap PinMap_SD[];
extern const PinMap PinMap_LCD[];
extern const PinMap PinMap_UI[];
extern const PinMap PinMap_UART_TX[];
extern const PinMap PinMap_UART_RX[];
extern const PinMap PinMap_CAN_RD[];
extern const PinMap PinMap_CAN_TD[];
extern const PinMap PinMap_USB_FS[];
extern const PinMap PinMap_USB_HS[];
extern const PinMap PinMap_I2C_SDA[];
extern const PinMap PinMap_I2C_SCL[];
extern const PinMap PinMap_OSC[];
extern const PinMap PinMap_SPI1[];
extern const PinMap PinMap_SPI2[];
extern const PinMap PinMap_SPI3[];
extern const PinMap PinMap_ESP[];
extern const PinMap PinMap_POWER[];
extern const PinMap PinMap_TMC_UART[];
extern const PinMap PinMap_TMC_SPI[];
extern const PinMap PinMap_MAX31865[];
extern const PinMap PinMap_DEBUG[];
extern const PinMap PinMap_BEEPER[];

#define PINMAP_ANALOGIN PinMap_ADC
#define PINMAP_ANALOGOUT PinMap_DAC
#define PINMAP_I2C_SDA PinMap_I2C_SDA
#define PINMAP_I2C_SCL PinMap_I2C_SCL
#define PINMAP_UART_TX PinMap_UART_TX
#define PINMAP_UART_RX PinMap_UART_RX
#define PINMAP_UART_CTS PinMap_UART_CTS
#define PINMAP_UART_RTS PinMap_UART_RTS
#define PINMAP_SPI_SCLK PinMap_SPI_SCLK
#define PINMAP_SPI_MOSI PinMap_SPI_MOSI 
#define PINMAP_SPI_MISO PinMap_SPI_MISO 
#define PINMAP_SPI_SSEL PinMap_SPI_SSEL
#define PINMAP_PWM PinMap_PWM
#define PINMAP_QSPI_DATA0 PinMap_QSPI
#define PINMAP_QSPI_DATA1 PinMap_QSPI
#define PINMAP_QSPI_DATA2 PinMap_QSPI
#define PINMAP_QSPI_DATA3 PinMap_QSPI
#define PINMAP_QSPI_SCLK PinMap_QSPI
#define PINMAP_QSPI_SSEL PinMap_QSPI
#define PINMAP_CAN_RD PinMap_CAN_RD
#define PINMAP_CAN_TD PinMap_CAN_TD

#endif
