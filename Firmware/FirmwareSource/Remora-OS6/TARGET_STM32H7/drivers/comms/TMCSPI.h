#ifndef TMCSPI_H
#define TMCSPI_H

#include "mbed.h"
#include "configuration.h"
#include "remora.h"

#include "stm32h7xx_hal.h"
#include <string>

#define ERR_OK 0
#define ERR 1
#define SPI_INIT_ERR -7
#define SPI_TRANSMIT_ERR -6
#define GPIO_WRITE_ERR -5
#define SPI_INIT_ERR -7

class TMCSPI
{
    private:

        SPI_TypeDef*        spiType;
        SPI_HandleTypeDef   spiHandle;
        HAL_StatusTypeDef   status;

        rxData_t            spiRxBuffer;
        uint8_t             rejectCnt;
        uint8_t             errorCode;
        bool                SPIdataError;
        
        PinName             CS_PIN;
        GPIO_TypeDef*        CS_PORT;
        
        

    public:

        TMCSPI(std::string cs_pin, std::string spiType); 
        uint8_t init(void);
        void start(void);
        void ResetCSPin(void);
        void SetCSPin(void);
        bool getStatus(void);
        void setStatus(bool);
        bool getError(void);
        void setError(bool error, uint8_t err);
        uint8_t getErrorCode();
        void spiWrite(uint8_t address, uint32_t value);
        uint32_t spiRead(uint8_t address);
        uint32_t tmc_read(uint8_t address);
        uint8_t spiSendReceive(uint8_t data);
        PinName stringToPinName(const std::string& gpio_str);
        SPI_TypeDef* stringToSPIType(const std::string& spiType_str);
        GPIO_TypeDef* stringToGPIO(const std::string& gpio_str);

};

#endif
