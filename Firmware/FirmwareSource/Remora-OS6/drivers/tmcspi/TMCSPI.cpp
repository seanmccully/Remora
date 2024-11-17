#include <algorithm>

#include "mbed.h"
#include "TMCSPI.h"
#include "arm_irq.h"



TMCSPI::TMCSPI(const char* miso_pin_str, const char* mosi_pin_str, const char* sck_pin_str, const char* cs_pin_str) :
        miso(Pin(miso_pin_str, GPIO_MODE_INPUT, GPIO_PULLUP, false)),
        mosi(Pin(mosi_pin_str, GPIO_MODE_OUTPUT_PP)),
        sck(Pin(sck_pin_str, GPIO_MODE_OUTPUT_PP)),
        cs_pin(Pin(cs_pin_str, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, true, 0)),
        swSPI(true) {
            sck.setPin(1);
        }



TMCSPI::TMCSPI(const char* spiType_str, const char* cs_pin_str) {
    cs_pin = Pin(cs_pin_str, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, true, 0);
    spiType = stringToSPIType(spiType_str);
    swSPI = false;
    cs_pin.setPin(1);
    init();
}


// Function to read from TMC5160
bool TMCSPI::isError(void)
{
    return lastStatus.driver_error;
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
        return SPI1; // Default if no number found
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

uint8_t TMCSPI::init() {

        if (spiType == SPI1) {
            spiH = new SPI(PA_7, PA_6, PA_5, cs_pin.pinToPinName(), {});
        }
#ifdef SPI2_BASE
        else if (spiType == SPI2) {
            spiH = new SPI(PB_15, PB_14, PB_13, cs_pin.pinToPinName(), {});
        }
#endif
#ifdef SPI3_BASE && defined TARGET_STM32F4
        else if (spiType == SPI3) {
            spiH = new SPI(PC_12, PC_11, PC_10, cs_pin.pinToPinName(), {});

        }
#endif
#ifdef SPI3_BASE
        else if (spiType == SPI3) {
            spiH = new SPI(PB_5, PB_4, PB_3, cs_pin.pinToPinName(), {});

        }
#endif
#ifdef SPI4_BASE
        else if (spiType == SPI4) {
            spiH = new SPI(PE_14, PE_13, PE_12, cs_pin.pinToPinName(), {});

        }
#endif
#ifdef SPI5_BASE
        else if (spiType == SPI5) {
            spiH = new SPI(PE_9, PE_8, PE_7, cs_pin.pinToPinName(), {});

        }
#endif

    spiH->format(8, 0);
    spiH->frequency(100000);
    spiH->deselect();
    return true;
}

bool TMCSPI::spi_flush_rx() {
#if defined(SPI_FLAG_FRLVL)
    uint8_t count = 0;
    uint32_t itflag = spiType->SR;
    __IO uint32_t tmpreg;
    while ( ((spiType->SR & SPI_FLAG_FRLVL) != SPI_RX_FIFO_0PACKET) || ((itflag & SPI_FLAG_RXWNE) != 0UL)) {
        count += 4UL;
        tmpreg = spiType->RXDR;
        UNUSED(tmpreg);
        if (count >= 16) {
            return true;
        }
    }
#endif
    return false;
}


void TMCSPI::swSPITransfer(TMC_spi_datagram *datagram, bool write) {
    uint8_t data[TMC_PACKET_SIZE] = {0};
    uint8_t temp[TMC_PACKET_SIZE] = {0}; // Temporary storage for reversing
    sck.setPin(1);
    cs_pin.setPin(0);

    // Prepare data for write or dummy for read
    data[0] = datagram->addr.value;

    // Write operation: Reverse bytes in payload
    for (int i = 0; i < 4; i++) {
   		data[i + 1] = datagram->payload.data[3 - i];
    }

    // Perform SPI transfer
    swSPITransferByte(data, TMC_PACKET_SIZE, write);

    if (write) {
        // Read operation: Reverse bytes back into payload
        for (int i = 0; i < 4; i++) {
            datagram->payload.data[3 - i] = data[i + 1];
        }
    }

    // Update status byte
    lastStatus.sr = data[0];

    cs_pin.setPin(1);
}

void TMCSPI::swSPITransferByte(uint8_t *data, size_t len, bool receive_data) {

    for (int t=0;t<len;t++) {
        uint_fast8_t inbuf = 0;
        uint8_t buf = *data;
        for (uint_fast8_t i=0;i<8;i++) {
            sck.togglePin();
            mosi.setPin( buf & 0x80);
            buf <<= 1;
            sck.togglePin();
            inbuf <<= 1;
            inbuf |= miso.readPin();
        }
        if (receive_data)
            *data = inbuf;
        data++;
    }
}

// Modified TMC register access methods
void TMCSPI::spiWrite(TMC_spi_datagram* dg) {

    // Prepare data buffer
    dg->addr.write = 1;
    TMC_spi_datagram *read = new TMC_spi_datagram(0,0);
    if (swSPI) {
        swSPITransfer(read,false);
        swSPITransfer(dg, false);
    } else {
#ifndef TARGET_STM32H7
        spiPrepare();
#endif
        spiTransfer(read,false);
        spiTransfer(dg, false);
    }
}

void TMCSPI::spiPrepare() {
    // The SPE bit must be disabled before changing CPOL/CPHA bits
    spiType->CR1 = cr1 & ~SPI_CR1_SPE;
    spiType->CR1; // Force flush of previous write
    spiType->CR1 = cr1;
}


void TMCSPI::spiRead(TMC_spi_datagram *dg) {

    dg->addr.write=0;
    TMC_spi_datagram *read = new TMC_spi_datagram(dg->addr.value,0);
    if (swSPI) {
        swSPITransfer(read, true);
        swSPITransfer(dg, true);
    }
    else {
#ifndef TARGET_STM32H7
        spiPrepare();
#endif
        spiTransfer(read, true);
        spiTransfer(dg, true);
    }

}


void TMCSPI::spiTransfer(TMC_spi_datagram* datagram, bool write)
{

    spiH->select();
    uint8_t _len = sizeof(datagram->payload.data) + sizeof(datagram->addr.value);
    uint8_t *rdata = new uint8_t[_len];
    uint8_t *data = new uint8_t[_len];

    *data = datagram->addr.value;
    for (uint8_t i=1;i<_len;i++) { // size of addr is 1
        *(data+i) = datagram->payload.data[i - 1];
    }

    notCompleted();
    spiH->transfer(data, sizeof(data), rdata, sizeof(rdata), callback(this, &TMCSPI::spiCallback), SPI_EVENT_COMPLETE);

    HAL_Delay(55);

    if (write) {
        for (uint8_t i=1;i<_len;i++) {
                datagram->payload.data[(_len - (i + 1))] = rdata[i];
        }
        lastStatus.sr = rdata[0];
    }
    spiH->deselect();

}

void TMCSPI::spiCallback(int spi_event) {
    if (spi_event == SPI_EVENT_COMPLETE)
        hasCompleted();
}

void TMCSPI::spiClockConfig() {
#if defined SPI1_BASE
    // Enable SPI clock
    if (spiType == SPI1) {
#if defined TARGET_STM32H7
        SET_BIT(RCC->PLLCFGR, RCC_PLL1_DIVQ);
#endif
        spiIRQ = SPI1_IRQn;
    }
#endif /* SPI_1 */

#if defined SPI2_BASE
    if (spiType == SPI2) {
#if defined TARGET_STM32H7
        SET_BIT(RCC->PLLCFGR, RCC_PLL1_DIVQ);
#endif
        spiIRQ = SPI2_IRQn;
    }
#endif /* SPI_IP_VERSION_V2 */

#if defined SPI3_BASE
    if (spiType == SPI3) {
#if defined TARGET_STM32H7
        SET_BIT(RCC->PLLCFGR, RCC_PLL1_DIVQ);
#endif
        spiIRQ = SPI3_IRQn;
    }
#endif
#if defined SPI4_BASE
    if (spiType == SPI4) {
        spiIRQ = SPI4_IRQn;
    }
#endif
#if defined SPI5_BASE
    if (spiType == SPI5) {
        spiIRQ = SPI5_IRQn;
    }
#endif
#if defined SPI6_BASE
    if (spiType == SPI6) {
        spiIRQ = SPI6_IRQn;
    }
#endif
}

