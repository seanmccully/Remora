// PeripheralPins.c
#include "SDMMCPins.h"

// SDIO pins - for STM32H743VIT
const PinMap PinMap_SD_CMD[] = {
    {PD_2,  SDIO, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_SDIO)},
    {NC,    NC,     0}
};

const PinMap PinMap_SD_CK[] = {
    {PC_12, SDIO, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF12_SDIO)},
    {NC,    NC,     0}
};

const PinMap PinMap_SD_DATA0[] = {
    {PC_8,  SDIO, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_SDIO)},
    {NC,    NC,     0}
};

const PinMap PinMap_SD_DATA1[] = {
    {PC_9,  SDIO, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_SDIO)},
    {NC,    NC,     0}
};

const PinMap PinMap_SD_DATA2[] = {
    {PC_10, SDIO, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_SDIO)},
    {NC,    NC,     0}
};

const PinMap PinMap_SD_DATA3[] = {
    {PC_11, SDIO, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_SDIO)},
    {NC,    NC,     0}
};

const PinMap PinMap_SD_DET[] = {
    {PC_14, SDIO, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF12_SDIO)},
    {NC,    NC,     0}
};
