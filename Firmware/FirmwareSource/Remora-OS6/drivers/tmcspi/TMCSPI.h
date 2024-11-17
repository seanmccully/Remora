#ifndef TMCSPI_H
#define TMCSPI_H

#include <atomic>
using namespace std;

#include "mbed.h"
#include "configuration.h"
#include "data.h"
#include "pin.h"
#include "gpioAPI.h"

typedef union {
    uint8_t sr;
    struct {
        uint8_t reset_flag    : 1,
             driver_error     : 1,
             sg2              : 1,
             standstill       : 1,
             velocity_reached : 1,
             position_reached : 1,
             status_stop_l    : 1,
             status_stop_t    : 1;
    };
} TMC_SPI_status_t;
typedef union  {
      uint8_t value;
      struct {
          uint8_t
          idx   :7,
          write :1;
      };
} TMC_addr_t;

typedef union {
      uint32_t value;
      uint8_t data[4];
} TMC_payload_t;


typedef struct TMC_spi_datagram {
    TMC_addr_t addr;
    TMC_payload_t payload;
      TMC_spi_datagram(uint8_t a, uint32_t v) { addr.value = a; payload.value = v; }
      TMC_spi_datagram() { addr.value = 0; payload.value = 0; }
} TMC_spi_datagram_t;

// Add these timing constants to TMCSPI.h header
#define CS_SETUP_TIME_US    4    // Time between CS falling edge and first clock edge
#define CS_HOLD_TIME_US     4    // Time between last clock edge and CS rising edge
#define CS_MIN_HIGH_TIME_US 10   // Minimum time CS must stay high between transfers



#define TMC5160_WRITE_MSK        0x80
#define TMC5160_ADDRESS_MSK      0x7F
#define TMC_PACKET_SIZE          (uint8_t)(40/8)
#define SPI_MODE				 0x1UL
#define SPI_RATE                 2000000

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



class TMCSPI : GPIOApi
{
    private:

		bool                swSPI;
        atomic<bool>        transferComplete{false};
        uint32_t            cr1; // Hold the CR1 Register;
        uint32_t            spiIRQ;
                                      //
        SPI_TypeDef*        spiType;
        TMC_SPI_status_t    lastStatus;
        SPI                 *spiH;

        Pin                 cs_pin;
        Pin                 sck;
        Pin                 mosi;
        Pin                 miso;

        [[nodiscard]] bool isCompleted() const { return transferComplete.load(std::memory_order_acquire);  }
        inline void hasCompleted() {  transferComplete.store(true, std::memory_order_release); }
        inline void notCompleted() {  transferComplete.store(false, std::memory_order_release); }

        void swSPITransfer(TMC_spi_datagram*, bool);
        void swSPITransferByte(uint8_t*,size_t,bool);

        void spiTransfer(TMC_spi_datagram*, bool);
        bool spi_flush_rx();
        void spiClockConfig();
        void spiPrepare();

        static inline void writeb(void *addr, uint8_t val) {
			barrier();
             *(volatile uint8_t *)addr = val;
        }
        static inline uint8_t readb(void *addr) {
			uint8_t val = *(volatile const uint8_t *)addr;
			barrier();
			return val;
        }

    public:

        TMCSPI(const char*, const char*);
        TMCSPI(const char*, const char*, const char*, const char*);
        uint8_t init(void);
        uint8_t swInit(void);

        bool isError(void);
        uint8_t getErrorCode();
        uint32_t tmc_read(uint8_t);
        void spiWrite(TMC_spi_datagram*);
        void spiRead(TMC_spi_datagram*);
        void setupPins();
        void spiCallback(int);

        PinName stringToPinName(const char*);
        SPI_TypeDef* stringToSPIType(const char*);
        void resetSPI();

        inline TMC_SPI_status_t getStatus() { return lastStatus; };
};

#endif
