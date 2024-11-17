#if ENABLE_REMORA_COMMS_SPI == 1
#ifndef REMORASPI_H
#define REMORASPI_H

#include "mbed.h"
#include "configuration.h"
#include "data.h"

#if defined TARGET_STM32F4
#include "stm32f4xx_hal.h"
#elif defined TARGET_SKRV3
#include "stm32h7xx_hal.h"
#elif defined TARGET_STM32F1
#include "stm32f1xx_hal.h"
#elif defined TARGET_STM32G0
#include "stm32f1xx_hal.h"
#endif



class RemoraComms
{
    private:

        SPI_TypeDef*        spiType;
        SPI_HandleTypeDef   spiHandle;
        DMA_HandleTypeDef   hdma_spi_tx;
        DMA_HandleTypeDef   hdma_spi_rx;
        DMA_HandleTypeDef   hdma_memtomem_dma2_stream1;
        HAL_StatusTypeDef   status;

        volatile rxData_t*  ptrRxData;
        volatile txData_t*  ptrTxData;
        rxData_t            spiRxBuffer;
        uint8_t             rejectCnt;
        bool                SPIdata;
        bool                SPIdataError;

        PinName             interruptPin;
        InterruptIn         slaveSelect;
        bool                sharedSPI;

        void processPacket(void);

    public:

        RemoraComms(volatile rxData_t*, volatile txData_t*, SPI_TypeDef*, PinName);
        void init(void);
        void start(void);
        bool getStatus(void);
        void setStatus(bool);
        bool getError(void);
        void setError(bool);

};

#endif
#endif
