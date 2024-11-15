#include <algorithm>

#include "mbed.h"
#include "TMCSPI.h"


#define LOG_FILE_NAME "/fs/log.txt"

TMCSPI::TMCSPI(const char* spiType_str, const char* cs_pin_str) : cs_pin(cs_pin_str, OUTPUT, PULLUP) {
		cs_port = cs_pin.getBus();
    spiType = stringToSPIType(spiType_str);
    init();

    // Enable DWT for precise timing
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    lastTransferTime = 0;
}

#include "TMCSPI.h"

// Add these timing constants to TMCSPI.h header
#define CS_SETUP_TIME_US    4    // Time between CS falling edge and first clock edge
#define CS_HOLD_TIME_US     4    // Time between last clock edge and CS rising edge
#define CS_MIN_HIGH_TIME_US 10   // Minimum time CS must stay high between transfers

void TMCSPI::SetCSPin() {
    HAL_GPIO_WritePin(cs_port, cs_pin.getPin(), GPIO_PIN_SET);
}

void TMCSPI::ResetCSPin() {
    // Ensure minimum CS high time has elapsed since last transfer
    cs_pin.initPin();
}

// Add a helper function for microsecond delays (if not already available)
void TMCSPI::delayMicroseconds(uint32_t us) {
    // Calculate the number of cycles needed for the delay
    uint32_t cycles = (SystemCoreClock / 1000000) * us;

    // Use DWT cycle counter for precise timing
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    while(DWT->CYCCNT < cycles);
}

// Function to send data via SPI
uint8_t TMCSPI::spiSendReceive(uint8_t data) {
    uint8_t receivedData;
    HAL_SPI_TransmitReceive(&spiHandle, &data, &receivedData, 1, HAL_MAX_DELAY);

    return receivedData;
}
// Function to read from TMC5160
bool TMCSPI::getError(void)
{
    return SPIdataError;
}

void TMCSPI::setError(bool error, uint8_t err)
{
    SPIdataError = error;
    errorCode = err;
    // TODO: handle error codes
    MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"TMCSPI Error",0x1243);
}
uint8_t TMCSPI::getErrorCode() {
    return errorCode;
}

PinName TMCSPI::stringToPinName(const char* gpio_str) {
    string upperStr = gpio_str;
    transform(upperStr.begin(), upperStr.end(), upperStr.begin(), ::toupper);

    // Check format starts with 'P'
    if (upperStr[0] != 'P') {
        return NC;
    }

    char port = upperStr[1];

    // Find pin number - handle both "PD5" and "PD_5" formats
    size_t numStart = (upperStr.find('_') != string::npos) ? 3 : 2;
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


SPI_TypeDef* TMCSPI::stringToSPIType(const char* spiType_str) {
    // Convert string to uppercase for consistent handling
    string upperStr = spiType_str;
    transform(upperStr.begin(), upperStr.end(), upperStr.begin(), ::toupper);

    // Remove any underscores or spaces
    string cleanStr;
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
#if defined(SPI2)
        case 2:
            return SPI2;
#endif
#if defined(SPI3)
        case 3:
            return SPI3;
#endif
#if defined(SPI4)
        case 4:
            return SPI4;
#endif
#if defined(SPI5)
        case 5:
            return SPI5;
#endif
#if defined(SPI6)
        case 6:
            return SPI6;
#endif
        default:
            return SPI1;
    }
}

GPIO_TypeDef* TMCSPI::stringToGPIO(const char* pinName) {
    // Extract port letter from pin string (e.g., "PD_5" -> 'D')
    if ((pinName[0] == 'P' || pinName[0] == 'p')) {
        char _ch = pinName[1];
        char port = toupper(_ch);
        switch (port) {
            case 'A':
                return GPIOA;
#if defined(GPIOB)
            case 'B':
                return GPIOB;
#endif
#if defined(GPIOC)
            case 'C':
                return GPIOC;
#endif
#if defined(GPIOD)
            case 'D':
                return GPIOD;
#endif
#if defined(GPIOE)
            case 'E':
                return GPIOE;
#endif
#if defined(GPIOF)
            case 'F':
                return GPIOF;
#endif
#if defined(GPIOG)
            case 'G':
                return GPIOG;
#endif
#if defined(GPIOH)
            case 'H':
                return GPIOH;
#endif
#if defined(GPIOI)
            case 'I':
                return GPIOI;
#endif
            default:
                return GPIOA; // Default fallback, though you might want to handle this error case differently
        }
     }
    return GPIOA; // Default fallback
 }

uint8_t TMCSPI::init() {
    // 1. Enable peripheral clocks first
    if (spiType == SPI1) {
        __HAL_RCC_SPI1_CLK_ENABLE();
    } else if (spiType == SPI4) {
        __HAL_RCC_SPI4_CLK_ENABLE();
    }

    // 2. Configure GPIO pins for SPI (keep existing GPIO setup code)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = cs_pin.getPin();
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(cs_port, &GPIO_InitStruct);

    // 3. Direct SPI configuration for H7 series
    SPI_TypeDef* spi = spiType;

    // Disable SPI first
    CLEAR_BIT(spi->CR1, SPI_CR1_SPE);

    // Configure CFG1 register
    MODIFY_REG(spi->CFG1,
        SPI_CFG1_MBR | SPI_CFG1_DSIZE | SPI_CFG1_FTHLV,
        (SPI_BAUDRATEPRESCALER_64)    | // Prescaler
        (7UL << SPI_CFG1_DSIZE_Pos)   | // 8-bit data size
        (0UL << SPI_CFG1_FTHLV_Pos));   // FIFO threshold 1 byte

    // Configure CFG2 register
    MODIFY_REG(spi->CFG2,
        SPI_CFG2_MASTER | SPI_CFG2_COMM | SPI_CFG2_SSM | SPI_CFG2_CPHA | SPI_CFG2_CPOL,
        SPI_CFG2_MASTER    | // Master mode
        (0x1UL << 24)      | // Full-duplex
        SPI_CFG2_SSM       | // Software slave management
        SPI_CFG2_CPHA      | // CPHA=1
        SPI_CFG2_CPOL);      // CPOL=1

    // Set control registers
    spi->CR1 = SPI_CR1_SSI;  // Internal slave select high

    // Clear all flags
    SET_BIT(spi->IFCR, 0xFFFFFFFF);

    // Enable SPI
    SET_BIT(spi->CR1, SPI_CR1_SPE);

    // Set CS pin high initially
    SetCSPin();
    delayMicroseconds(CS_MIN_HIGH_TIME_US);

    return ERR_OK;
}

bool TMCSPI::spiSendReceiveWithRetry(uint8_t data, uint8_t* receivedData) {
    SPI_TypeDef* spi = spiType;
    uint32_t startTime = DWT->CYCCNT;
    uint8_t received;

    // Disable SPI before setting transfer size
    CLEAR_BIT(spi->CR1, SPI_CR1_SPE);

    // Clear all flags
    SET_BIT(spi->IFCR, 0xFFFFFFFF);

    // Set transfer size to 1 byte
    MODIFY_REG(spi->CR2, SPI_CR2_TSIZE, 1);

    // Enable SPI and start transfer
    SET_BIT(spi->CR1, SPI_CR1_SPE);
    SET_BIT(spi->CR1, SPI_CR1_CSTART);

    // Wait for TXP (Tx empty) flag
    while (!(spi->SR & SPI_SR_TXP)) {
        if ((DWT->CYCCNT - startTime) / (SystemCoreClock / 1000000) > SPI_TIMEOUT_US) {
            return false;
        }
    }

    // Write data
    *((__IO uint8_t*)&spi->TXDR) = data;

    // Wait for RXWNE or RXPLVL (Rx not empty) flag
    while (!(spi->SR & (SPI_SR_RXWNE | SPI_SR_RXPLVL))) {
        if ((DWT->CYCCNT - startTime) / (SystemCoreClock / 1000000) > SPI_TIMEOUT_US) {
            return false;
        }
    }

    // Read received data
    received = *((__IO uint8_t*)&spi->RXDR);

    // Wait for end of transfer
    while (!(spi->SR & SPI_SR_EOT)) {
        if ((DWT->CYCCNT - startTime) / (SystemCoreClock / 1000000) > SPI_TIMEOUT_US) {
            return false;
        }
    }

    // Clear flags
    SET_BIT(spi->IFCR, 0xFFFFFFFF);

    // Store received data if pointer is provided
    if (receivedData) {
        *receivedData = received;
    }

    return true;
}

void TMCSPI::spiWrite(uint8_t address, uint32_t value) {
    ResetCSPin();  // CS low
    delayMicroseconds(CS_SETUP_TIME_US);

    // Send address byte (with write flag set)
    spiSendReceiveWithRetry(address | 0x80, nullptr);

    // Send data bytes
    spiSendReceiveWithRetry((value >> 24) & 0xFF, nullptr);
    spiSendReceiveWithRetry((value >> 16) & 0xFF, nullptr);
    spiSendReceiveWithRetry((value >> 8) & 0xFF, nullptr);
    spiSendReceiveWithRetry(value & 0xFF, nullptr);

    delayMicroseconds(CS_HOLD_TIME_US);
    SetCSPin();  // CS high

    // Minimum time before next transfer
    delayMicroseconds(CS_MIN_HIGH_TIME_US);
}

uint32_t TMCSPI::spiRead(uint8_t address) {
    uint32_t value = 0;
    uint8_t received;

    ResetCSPin();  // CS low
    delayMicroseconds(CS_SETUP_TIME_US);

    // Send address byte (with read flag clear)
    spiSendReceiveWithRetry(address & 0x7F, nullptr);

    // Read data bytes
    spiSendReceiveWithRetry(0, &received);
    value = ((uint32_t)received << 24);
    spiSendReceiveWithRetry(0, &received);
    value |= ((uint32_t)received << 16);
    spiSendReceiveWithRetry(0, &received);
    value |= ((uint32_t)received << 8);
    spiSendReceiveWithRetry(0, &received);
    value |= received;

    delayMicroseconds(CS_HOLD_TIME_US);
    SetCSPin();  // CS high

    // Minimum time before next transfer
    delayMicroseconds(CS_MIN_HIGH_TIME_US);

    return value;
}
