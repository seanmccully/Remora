#include <algorithm> 

#include "mbed.h"
#include "TMCSPI.h"


#define LOG_FILE_NAME "/fs/log.txt"

TMCSPI::TMCSPI(const char* cs_pin_str, const char* spiType_str) {
    CS_PIN = stringToPinName(cs_pin_str);;
    CS_PORT = stringToGPIO(cs_pin_str);
    spiType = stringToSPIType(spiType_str);
    spiHandle.Instance = spiType;
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
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
    // Ensure minimum CS high time between transfers
    delayMicroseconds(CS_MIN_HIGH_TIME_US);
}

void TMCSPI::ResetCSPin() {
    // Ensure minimum CS high time has elapsed since last transfer
    delayMicroseconds(CS_MIN_HIGH_TIME_US);
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
    // Add setup delay before first clock edge
    delayMicroseconds(CS_SETUP_TIME_US);
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

// Add this to the class initialization
uint8_t TMCSPI::init() {

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Enable DWT for precise timing
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    // Ensure CS starts high with minimum high time
    SetCSPin();
    delayMicroseconds(CS_MIN_HIGH_TIME_US);


    /**SPI4 GPIO Configuration
    PE11      ------> SPI4_NSS
    PE14     ------> SPI4_SCK
    PE15     ------> SPI4_MISO
    PE13     ------> SPI4_MOSI
    */

    if (HAL_SPI_Init(&spiHandle) != HAL_OK) {
        // Initialization Error
        setError(ERR, SPI_INIT_ERR);
        return getErrorCode();
    }

    // Enable GPIO clock
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Set CS high initially
    GPIO_InitStruct.Pin     = CS_PIN;
    GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull    = GPIO_NOPULL;
    GPIO_InitStruct.Speed   = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CS_PORT, &GPIO_InitStruct);
    ResetCSPin();


    __HAL_RCC_SPI4_CLK_ENABLE();

    spiHandle.Instance = spiType;
    spiHandle.Init.Mode              = SPI_MODE_MASTER;
    spiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    spiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    spiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    spiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    spiHandle.Init.NSS               = SPI_NSS_SOFT;
    spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

    HAL_SPI_Init(&spiHandle);

    return ERR_OK;
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

// Helper function to enforce minimum delay between transfers
void TMCSPI::enforceTransferDelay() {
    uint32_t currentTime = DWT->CYCCNT;
    uint32_t elapsedTime = (currentTime - lastTransferTime) / (SystemCoreClock / 1000000);

    if (elapsedTime < SPI_MIN_DELAY_US) {
        delayMicroseconds(SPI_MIN_DELAY_US - elapsedTime);
    }
    lastTransferTime = DWT->CYCCNT;
}



void TMCSPI::spiWrite(uint8_t address, uint32_t value) {
    enforceTransferDelay();

    ResetCSPin();  // CS low with setup time
    delayMicroseconds(CS_SETUP_TIME_US);

    // Write command and data with timing guarantees
    spiSendReceiveWithRetry(address | 0x80);
    spiSendReceiveWithRetry((value >> 24) & 0xff);
    spiSendReceiveWithRetry((value >> 16) & 0xff);
    spiSendReceiveWithRetry((value >> 8) & 0xff);
    spiSendReceiveWithRetry(value & 0xff);

    delayMicroseconds(CS_HOLD_TIME_US);
    SetCSPin();

    // Ensure minimum time before next transfer
    lastTransferTime = DWT->CYCCNT;
}

// Modified read function with timing guarantees
uint32_t TMCSPI::spiRead(uint8_t address) {
    enforceTransferDelay();

    uint32_t value = 0;
    uint8_t retry_count = 0;
    bool success = false;

    while (!success && retry_count < SPI_MAX_RETRIES) {
        ResetCSPin();
        delayMicroseconds(CS_SETUP_TIME_US);

        // Send read command
        if (!spiSendReceiveWithRetry(address & 0x7F)) {
            retry_count++;
            delayMicroseconds(SPI_RETRY_DELAY_US);
            continue;
        }

        // Read data bytes with timing guarantees
        uint8_t responses[4];
        bool readSuccess = true;

        for (int i = 0; i < 4; i++) {
            uint8_t response;
            if (!spiSendReceiveWithRetry(0x00, &response)) {
                readSuccess = false;
                break;
            }
            responses[i] = response;
        }

        if (readSuccess) {
            value = ((uint32_t)responses[0] << 24) |
                    ((uint32_t)responses[1] << 16) |
                    ((uint32_t)responses[2] << 8) |
                    responses[3];
            success = true;
        } else {
            retry_count++;
            delayMicroseconds(SPI_RETRY_DELAY_US);
        }

        delayMicroseconds(CS_HOLD_TIME_US);
        SetCSPin();
    }

    if (!success) {
        setError(ERR, SPI_TRANSMIT_ERR);
    }

    lastTransferTime = DWT->CYCCNT;
    return value;
}

// Enhanced SPI transfer function with timeout and error handling
bool TMCSPI::spiSendReceiveWithRetry(uint8_t data, uint8_t* receivedData) {
    uint8_t received;
    uint32_t startTime = DWT->CYCCNT;
    HAL_StatusTypeDef status;

    do {
        status = HAL_SPI_TransmitReceive(&spiHandle, &data, &received, 1, HAL_MAX_DELAY);

        if (status == HAL_OK) {
            if (receivedData) *receivedData = received;
            return true;
        }

        // Check for timeout
        uint32_t elapsedTime = (DWT->CYCCNT - startTime) / (SystemCoreClock / 1000000);
        if (elapsedTime > SPI_TIMEOUT_US) {
            setError(ERR, SPI_TRANSMIT_ERR);
            return false;
        }

        delayMicroseconds(10); // Short delay before retry
    } while (true);

}
