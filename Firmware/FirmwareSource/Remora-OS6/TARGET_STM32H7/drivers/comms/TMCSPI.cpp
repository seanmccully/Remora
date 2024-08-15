#include "mbed.h"
#include "TMCSPI.h"

#include "stm32h7xx_hal.h"

TMCSPI::TMCSPI(std::string cs_pin, std::string spiType) {
    this->CS_PIN = this->stringToPinName(cs_pin);
    this->CS_PORT = this->stringToGPIO(spiType);
    this->spiType = this->stringToSPIType(spiType);
    this->spiHandle.Instance = this->spiType;
    this->init();
}

PinName TMCSPI::stringToPinName(const std::string& gpio_str) {
    if (gpio_str == "PD_5" || gpio_str == "PD5") {
        return PD_5;
    } else if (gpio_str == "PD_1" || gpio_str == "PD1") {
        return PD_1;
    } else if (gpio_str == "PD_0" || gpio_str == "PD0") {
        return PD_0;
    } else if (gpio_str == "PE_1" || gpio_str == "PE1") {
        return PE_1;
    } else {
        return PD_5;
    }
}

SPI_TypeDef* TMCSPI::stringToSPIType(const std::string& spiType_str) {
    if (spiType_str == "SPI1" || spiType_str == "SPI_1") {
        return SPI1;
    } else if (spiType_str == "SPI2" || spiType_str == "SPI_2") {
        return SPI2;
    } else if (spiType_str == "SPI3" || spiType_str == "SPI_3") {
        return SPI3;
    } else if (spiType_str == "SPI4" || spiType_str == "SPI_4") {
        return SPI4;
    } else {
        return SPI4;
    }
}

GPIO_TypeDef* TMCSPI::stringToGPIO(const std::string& gpio_str) {
    if (gpio_str == "SPI1") {
        return GPIOA;
    } else if (gpio_str == "SPI2") {
        return GPIOB;
    } else if (gpio_str == "SPI3") {
        return GPIOC;
    } else if (gpio_str == "SPI4") {
        return GPIOD;
    } else {
        return GPIOE;
    }
}


uint8_t TMCSPI::init()
{
    printf("Initialising SPI1 slave\n");

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /**SPI4 GPIO Configuration
    PE11      ------> SPI4_NSS
    PE14     ------> SPI4_SCK
    PE15     ------> SPI4_MISO
    PE13     ------> SPI4_MOSI
    */

    if (HAL_SPI_Init(&this->spiHandle) != HAL_OK) {
        // Initialization Error
        this->setError(ERR, SPI_INIT_ERR);
        return getErrorCode();
    }

    // Enable GPIO clock
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Set CS high initially
    GPIO_InitStruct.Pin     = this->CS_PIN;
    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_NOPULL;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(this->CS_PORT, &GPIO_InitStruct);
    this->ResetCSPin();


    __HAL_RCC_SPI4_CLK_ENABLE();

    this->spiHandle.Instance = this->spiType;
    this->spiHandle.Init.Mode              = SPI_MODE_MASTER;
    this->spiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    this->spiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    this->spiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    this->spiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    this->spiHandle.Init.NSS               = SPI_NSS_SOFT;
    this->spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

    HAL_SPI_Init(&this->spiHandle);

    return ERR_OK;
}

// Function to send data via SPI
uint8_t TMCSPI::spiSendReceive(uint8_t data) {
    uint8_t receivedData;
    HAL_SPI_TransmitReceive(&this->spiHandle, &data, &receivedData, 1, HAL_MAX_DELAY);

    return receivedData;
}

// Function to write to TMC5160
void TMCSPI::spiWrite(uint8_t address, uint32_t value) {
    this->ResetCSPin();


    spiSendReceive(address | 0x80); // write command
    spiSendReceive((value >> 24) & 0xff);
    spiSendReceive((value >> 16) & 0xff);
    spiSendReceive((value >> 8) & 0xff);
    spiSendReceive(value & 0xff);

    this->SetCSPin();

}

// Function to read from TMC5160
uint32_t TMCSPI::spiRead(uint8_t address) {
    uint32_t value = 0;
    this->ResetCSPin();

    spiSendReceive(address & 0x7f); // read command
    value |= spiSendReceive(0x00) << 24;
    value |= spiSendReceive(0x00) << 16;
    value |= spiSendReceive(0x00) << 8;
    value |= spiSendReceive(0x00);

    this->SetCSPin();
    return value;
}

void TMCSPI::SetCSPin() {
    HAL_GPIO_WritePin(this->CS_PORT, this->CS_PIN, GPIO_PIN_SET);
}

void TMCSPI::ResetCSPin() {
    HAL_GPIO_WritePin(this->CS_PORT, this->CS_PIN, GPIO_PIN_RESET);
}

bool TMCSPI::getError(void)
{
    return this->SPIdataError;
}

void TMCSPI::setError(bool error, uint8_t err)
{
    this->SPIdataError = error;
    this->errorCode = err;
}
uint8_t TMCSPI::getErrorCode() {
    return this->errorCode;
}
