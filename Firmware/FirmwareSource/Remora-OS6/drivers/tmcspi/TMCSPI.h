#ifndef TMCSPI_H
#define TMCSPI_H

using namespace std;

#include "mbed.h"
#include "configuration.h"
#include "data.h"
#include "pin.h"

#if defined TARGET_STM32F4
#include "stm32f4xx_hal.h"
#elif defined TARGET_SKRV3
#include "stm32h7xx_hal.h"
#elif defined TARGET_STM32F1
#include "stm32f1xx_hal.h"
#elif defined TARGET_STM32G0
#include "stm32f1xx_hal.h"
#endif

#define ERR_OK 0
#define ERR 1
#define SPI_INIT_ERR -7
#define SPI_TRANSMIT_ERR -6
#define GPIO_WRITE_ERR -5
#define SPI_INIT_ERR -7

#define SPI_MIN_DELAY_US          10   // Minimum time between consecutive transfers
#define SPI_RETRY_DELAY_US       100   // Delay before retry on failed transfer
#define SPI_TIMEOUT_US          1000   // Maximum wait time for a transfer
#define SPI_POST_RESET_DELAY_US   50   // Delay after CS reset before next transfer
#define SPI_MAX_RETRIES           3    // Maximum number of retry attempts

class TMCSPI
{
    private:

        uint32_t lastTransferTime;    // Timestamp of last transfer
        SPI_TypeDef*        spiType;
        SPI_HandleTypeDef   spiHandle;
        HAL_StatusTypeDef   status;

        rxData_t            spiRxBuffer;
        uint8_t             rejectCnt;
        uint8_t             errorCode;
        bool                SPIdataError;

        Pin                  cs_pin;
        GPIO_TypeDef*        cs_port;



    public:

        TMCSPI(const char* cs_pin, const char* spiType_str);
        void SetCSPin(void);
        void delayMicroseconds(uint32_t us);
        uint8_t init(void);
        void start(void);
        void ResetCSPin(void);
        bool getStatus(void);
        void setStatus(bool);
        bool getError(void);
        void setError(bool error, uint8_t err);
        uint8_t getErrorCode();
        void spiWrite(uint8_t address, uint32_t value);
        uint32_t spiRead(uint8_t address);
        uint32_t tmc_read(uint8_t address);
        uint8_t spiSendReceive(uint8_t data);
        bool spiSendReceiveWithRetry(uint8_t data, uint8_t* receivedData = nullptr);
        PinName stringToPinName(const char* gpio_str);
        SPI_TypeDef* stringToSPIType(const const char* spiType_str);
        GPIO_TypeDef* stringToGPIO(const char*  gpio_str);
				void resetSPI();
				HAL_SPI_StateTypeDef getSPIState();

};

#endif
