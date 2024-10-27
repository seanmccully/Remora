// PeripheralPins.h
#ifndef _SDMMCPINS_H
#define _SDMMCPINS_H

#include "pinmap.h"
#include "PeripheralNames.h"

// Add to header file or in a configuration file
//
// /* SDIO Configuration defines */
#define SDIO_CLOCK_EDGE_RISING          0x00000000U
#define SDIO_CLOCK_EDGE_FALLING         SDIO_CLKCR_NEGEDGE
//
#define SDIO_CLOCK_BYPASS_DISABLE       0x00000000U
#define SDIO_CLOCK_BYPASS_ENABLE        SDIO_CLKCR_BYPASS
//
#define SDIO_CLOCK_POWER_SAVE_DISABLE   0x00000000U
#define SDIO_CLOCK_POWER_SAVE_ENABLE    SDIO_CLKCR_PWRSAV
//
#define SDIO_BUS_WIDE_1B                0x00000000U
#define SDIO_BUS_WIDE_4B                SDIO_CLKCR_WIDBUS_0
#define SDIO_BUS_WIDE_8B                SDIO_CLKCR_WIDBUS_1
//
#define SDIO_HARDWARE_FLOW_CONTROL_DISABLE  0x00000000U
#define SDIO_HARDWARE_FLOW_CONTROL_ENABLE   SDIO_CLKCR_HWFC_EN
// Maximum frequencies for different modes
#define SDIO_INIT_CLK_DIV    ((uint8_t)0x76)     // 400 KHz for initialization (48MHz / (118 + 2) ≈ 400KHz)
#define SDIO_NORMAL_CLK_DIV  ((uint8_t)0x0)      // 48MHz / (0 + 2) = 24MHz for normal operation
#define SDIO_HIGH_CLK_DIV    ((uint8_t)0x0)      // 48MHz / (0 + 2) = 24MHz for high speed

// Frequency values in Hz
#define SDIO_INIT_FREQ       400000      // 400 KHz for initialization
#define SDIO_NORMAL_FREQ     24000000    // 24 MHz for normal operation
#define SDIO_HIGH_FREQ       24000000    // 24 MHz for high speed mode


// /* SDIO Transfer state defines */
#define SDIO_TRANSFER_OK                ((uint8_t)0x00)
#define SDIO_TRANSFER_BUSY              ((uint8_t)0x01)
#define SDIO_TRANSFER_ERROR             ((uint8_t)0x02)
//
// /* SDIO Timeout defines */
#define SDIO_DATATIMEOUT               ((uint32_t)0xFFFFFFFF)
#define SDIO_INITIALIZATION_TIMEOUT     ((uint32_t)5000)    /* 5s initialization timeout */
//
// /* SDIO IRQ defines */
#define SDIO_IRQ_PRIORITY              0x0F    /* SDIO interrupt priority */
#define SDIO_DMA_IRQ_PRIORITY          0x0F    /* SDIO DMA interrupt priority */
//
// /* SDIO DMA defines */
#define SDIO_DMA_STREAM                DMA2_Stream3
#define SDIO_DMA_CHANNEL               DMA_CHANNEL_4

// SDMMC Pins
extern const PinMap PinMap_SD_CMD[];
extern const PinMap PinMap_SD_CK[];
extern const PinMap PinMap_SD_DATA0[];
extern const PinMap PinMap_SD_DATA1[];
extern const PinMap PinMap_SD_DATA2[];
extern const PinMap PinMap_SD_DATA3[];
extern const PinMap PinMap_SD_DET[];

#endif

