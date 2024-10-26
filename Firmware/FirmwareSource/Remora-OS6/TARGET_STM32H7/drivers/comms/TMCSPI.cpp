#include <algorithm> 

#include "mbed.h"
#include "TMCSPI.h"

#include "stm32h7xx_hal.h"

TMCSPI::TMCSPI(std::string cs_pin, std::string spiType) {
    this->CS_PIN = this->stringToPinName(cs_pin);
    this->CS_PORT = this->stringToGPIO(cs_pin);
    this->spiType = this->stringToSPIType(spiType);
    this->spiHandle.Instance = this->spiType;
    this->init();
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

PinName TMCSPI::stringToPinName(const std::string& gpio_str) {
    // Check minimum length requirement
    if (gpio_str.length() < 3) {
        return NC;
    }

    // Convert string to uppercase for consistent handling
    std::string upperStr = gpio_str;
    std::transform(upperStr.begin(), upperStr.end(), upperStr.begin(), ::toupper);

    // Check format starts with 'P'
    if (upperStr[0] != 'P') {
        return NC;
    }

    char port = upperStr[1];
    
    // Find pin number - handle both "PD5" and "PD_5" formats
    size_t numStart = (upperStr.find('_') != std::string::npos) ? 3 : 2;
    if (numStart >= upperStr.length()) {
        return NC;
    }

    // Convert pin number string to integer
    const char* numStr = upperStr.c_str() + numStart;
    char* endPtr;
    long pinNum = strtol(numStr, &endPtr, 10);
    
    // Check conversion was successful and complete
    if (endPtr == numStr || *endPtr != '\0' || pinNum < 0 || pinNum > 15) {
        return NC;
    }

    // In STM32/mbed, pins are defined as: PIN_PORT(port,pin) = (port << 4) | pin
    // where port is 0-9 for A-J
    uint32_t portNum;
    switch (port) {
        case 'A': portNum = 0; break;
        case 'B': portNum = 1; break;
        case 'C': portNum = 2; break;
        case 'D': portNum = 3; break;
        case 'E': portNum = 4; break;
        case 'F': portNum = 5; break;
        case 'G': portNum = 6; break;
        case 'H': portNum = 7; break;
        case 'I': portNum = 8; break;
        case 'J': portNum = 9; break;
        default: return NC;
    }

    // Calculate pin value: (port << 4) | pin
    return static_cast<PinName>((portNum << 4) | pinNum);
}


SPI_TypeDef* TMCSPI::stringToSPIType(const std::string& spiType_str) {
    // Convert string to uppercase for consistent handling
    std::string upperStr = spiType_str;
    std::transform(upperStr.begin(), upperStr.end(), upperStr.begin(), ::toupper);

    // Remove any underscores or spaces
    std::string cleanStr;
    for (char c : upperStr) {
        if (c != '_' && c != ' ') {
            cleanStr += c;
        }
    }

    // Find first digit
    size_t numStart = 0;
    while (numStart < cleanStr.length() && !isdigit(cleanStr[numStart])) {
        numStart++;
    }

    if (numStart >= cleanStr.length()) {
        return SPI4; // Default if no number found
    }

    // Convert to number using strtol
    const char* numStr = cleanStr.c_str() + numStart;
    char* endPtr;
    long spiNum = strtol(numStr, &endPtr, 10);

    // Check conversion was successful
    if (endPtr == numStr || *endPtr != '\0') {
        return SPI4;
    }

    // Map SPI number to peripheral
    switch (spiNum) {
        case 1:
            return SPI1;
        case 2:
            return SPI2;
        case 3:
            return SPI3;
        case 4:
            return SPI4;
        case 5:
            return SPI5;
        case 6:
            return SPI6;
        default:
            return SPI4;
    }
}

GPIO_TypeDef* TMCSPI::stringToGPIO(const std::string& pinName) {
    // Extract port letter from pin string (e.g., "PD_5" -> 'D')
    if (pinName.length() >= 2 && (pinName[0] == 'P' || pinName[0] == 'p')) {
        char _ch = pinName[1];
        char port = toupper(_ch);
        switch (port) {
            case 'A':
                return GPIOA;
            case 'B':
                return GPIOB;
            case 'C':
                return GPIOC;
            case 'D':
                return GPIOD;
            case 'E':
                return GPIOE;
            case 'F':
                return GPIOF;
            case 'G':
                return GPIOG;
            case 'H':
                return GPIOH;
            case 'I':
                return GPIOI;
            default:
                return GPIOD; // Default fallback, though you might want to handle this error case differently
        }
     }
    return GPIOD; // Default fallback
 }
