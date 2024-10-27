// SDMMCBlockDevice.h
#ifndef MBED_SDMMC_BLOCK_DEVICE_H
#define MBED_SDMMC_BLOCK_DEVICE_H

#include "mbed.h"
#include "platform/PlatformMutex.h"
#include "blockdevice/BlockDevice.h"

#include "pinmap.h"
#include "board.h"

#include "stm32_hal_legacy.h"
#if defined TARGET_STM32F4
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_sd.h"  // Added for SD specific macros
#include "stm32f4xx_ll_sdmmc.h" // Added for low-level SDMMC control
#elif defined TARGET_STM32H7
#include "stm32h7xx.h"
#elif defined TARGET_STM32F1
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_sd.h"  // Added for SD specific macros
#include "stm32f1xx_ll_sdmmc.h" // Added for low-level SDMMC control
#elif defined TARGET_STM32G0
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_sd.h"  // Added for SD specific macros
#include "stm32g0xx_ll_sdmmc.h" // Added for low-level SDMMC control
#endif

#define STM_GPIO_PIN(X) ((uint16_t)(1<<STM_PIN(X)))
/** SD Card block device driver class for STM32H7 using SDMMC peripheral
 */

extern void HAL_SD_MspInit(SD_HandleTypeDef *hsd);
extern void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd);

class SDMMCBlockDevice : public BlockDevice {
public:
    /** Supported SD Card types */
    enum CardType {
        SDCARD_NONE = 0,
        SDCARD_V1 = 1,    ///< v1.x Standard Capacity
        SDCARD_V2 = 2,    ///< v2.x Standard Capacity
        SDCARD_V2HC = 3,  ///< v2.x High/Extended Capacity
        SDCARD_SECURE = 4 ///< Secure Digital Card
    };

    /** Creates an SDMMCBlockDevice
     *
     * @param d0       Data line 0 pin
     * @param d1       Data line 1 pin
     * @param d2       Data line 2 pin
     * @param d3       Data line 3 pin
     * @param cmd      Command pin
     * @param clk      Clock pin
     * @param cd       Card detect pin (optional, default NC)
     */
    SDMMCBlockDevice(PinName detect_pin);
    
    virtual int sync();
    virtual ~SDMMCBlockDevice();

    /** Initialize a block device
     *
     *  @return         0 on success or a negative error code on failure
     */
    int init();

    /** Deinitialize a block device
     *
     *  @return         0 on success or a negative error code on failure
     */
    int deinit();

    /** Read blocks from a block device
     *
     *  @param buffer   Buffer to write blocks to
     *  @param addr     Address of block to begin reading from
     *  @param size     Size to read in bytes, must be a multiple of read block size
     *  @return         0 on success, negative error code on failure
     */
    int read(void *buffer, bd_addr_t addr, bd_size_t size);

    /** Write blocks to a block device
     *
     *  @param buffer   Buffer of data to write to blocks
     *  @param addr     Address of block to begin writing to
     *  @param size     Size to write in bytes, must be a multiple of program block size
     *  @return         0 on success, negative error code on failure
     */
    int program(const void *buffer, bd_addr_t addr, bd_size_t size);


    /** Mark blocks as no longer in use
     *
     *  This function provides a hint to the underlying block device that a region of blocks
     *  is no longer in use and may be erased without side effects. Erase must still be called
     *  before programming, but trimming allows flash-translation-layers to schedule erases when
     *  the device is not busy.
     *
     *  @param addr     Address of block to mark as unused
     *  @param size     Size to mark as unused in bytes, must be a multiple of erase block size
     *  @return         BD_ERROR_OK(0) - success
     *                  SD_BLOCK_DEVICE_ERROR_NO_DEVICE - device (SD card) is missing or not connected
     *                  SD_BLOCK_DEVICE_ERROR_CRC - crc error
     *                  SD_BLOCK_DEVICE_ERROR_PARAMETER - invalid parameter
     *                  SD_BLOCK_DEVICE_ERROR_UNSUPPORTED - unsupported command
     *                  SD_BLOCK_DEVICE_ERROR_NO_INIT - device is not initialized
     *                  SD_BLOCK_DEVICE_ERROR_ERASE - erase error
     */
    virtual int trim(mbed::bd_addr_t addr, mbed::bd_size_t size);
    /** Erase blocks on a block device
     *
     *  @param addr     Address of block to begin erasing
     *  @param size     Size to erase in bytes, must be a multiple of erase block size
     *  @return         0 on success, negative error code on failure
     */
    virtual int erase(bd_addr_t addr, bd_size_t size);

    /** Get the size of a readable block
     *
     *  @return         Size of a readable block in bytes
     */
    bd_size_t get_read_size() const;

    /** Get the size of a programmable block
     *
     *  @return         Size of a programmable block in bytes
     */
    bd_size_t get_program_size() const;

    /** Get the size of an erasable block
     *
     *  @return         Size of an erasable block in bytes
     */
     virtual int get_erase_value() const;
     virtual bd_size_t get_erase_size(bd_addr_t addr) const;
     virtual bd_size_t get_erase_size() const;

    /** Get the total size of the underlying device
     *
     *  @return         Size of the underlying device in bytes
     */
     bd_size_t size() const;

    /** Enable or disable debugging
     *
     *  @param dbg        State of debugging
     */
    virtual void debug(bool dbg);

    /** Set the transfer frequency
     *
     *  @param freq     Transfer frequency
     *  @note Max frequency supported is 25MHZ
     */
    virtual int frequency(uint64_t freq);

    /** Get current frequency
     *
     * @return         Frequency in Hz
     */
    virtual uint64_t frequency() const;

    /** Get the card type
     *
     *  @return         CardType enum value
     */
    virtual CardType card_type();

    /** Check if a card is present
     *
     *  @return         true if a card is present
     */
    virtual bool is_present();


    const char *get_type() const;
    
private:
    // SDMMC HAL handle
    SD_HandleTypeDef _sd_handle;
    HAL_SD_CardInfoTypeDef _card_info;
	  uint8_t BSP_SD_GetCardState(void);
    
    // Pin configuration
    PinMap pinMap_SD;
    PinName _cd;

    
    bool _is_initialized;
    bd_size_t _block_size;
    bd_size_t _erase_size;
    bd_size_t _capacity_in_blocks;
		uint32_t _init_ref_count;
    
    uint64_t _freq;
    static uint32_t _calculate_clock_divider(uint64_t freq);
		bool is_valid_read(bd_addr_t addr, bd_size_t size);
		bool is_valid_write(bd_addr_t addr, bd_size_t size);
		bool is_valid_trim(bd_addr_t addr, bd_size_t size);
    /** Initialize SDMMC peripheral
     *
     *  @return         0 on success or a negative error code on failure
     */
    int _initialise_card();
    
    /** Configure SDMMC pins
     */
    //void _init_sdmmc_pins();
    void _deinit_sdmmc_pins(SD_HandleTypeDef *hsd);
    
    /** Convert HAL errors to BlockDevice errors
     */
    PlatformMutex _mutex;
    int _convert_error(HAL_StatusTypeDef status);
		virtual void lock() {
			_mutex.lock();
		}
		virtual void unlock() {
			_mutex.unlock();
		}
};
#endif // MBED_SDMMC_BLOCK_DEVICE_H

