
// SDMMCBlockDevice.cpp
#include "SDMMCBlockDevice.h"
#include "stm32_hal_legacy.h"
#include "pinmap.h"

// This is a common pin on BTT Boards, but TODO
#define SD_CK_PIN PC_12
// Add these defines at the top of the file
#define SD_CMD_FLUSH_CACHE    32
#define SD_TIMEOUT            5000  // 5 seconds timeout
#define SD_TRANSFER_OK      0
#define SD_TRANSFER_BUSY    1
#define SD_TRANSFER_ERROR   2

#define BD_ERROR_NO_INIT -1
#define BD_ERROR_PARAMETER -2
#define BD_ERROR_TIMEOUT -3
// Add these defines at the top of the file if not already present

extern GPIO_TypeDef* Set_GPIO_Clock(uint32_t port_idx);

#if defined SDMMC_FLAG_CMDACT
#define FLAG_CMDACT SDMMC_FLAG_CMDACT
#elif defined SDIO_FLAG_CMDACT
#define FLAG_CMDACT SDIO_FLAG_CMDACT
#endif
#if defined RCC_PERIPHCLK_SDIO
#define SD_PERIPHCLK RCC_PERIPHCLK_SDIO
#elif defined RCC_PERIPHCLK_SDMMC
#define SD_PERIPHCLK RCC_PERIPHCLK_SDMMC
#elif defined RCC_PERIPHCLK_SDMMC1
#define SD_PERIPHCLK RCC_PERIPHCLK_SDMMC1
#endif
    // Once in transfer state, ensure the data is flushed
#ifndef SDIO_INIT_FREQ
#define INIT_FREQ SDMMC_INIT_CLK_DIV
#else
#define INIT_FREQ SDIO_INIT_FREQ
#endif

#if defined(SDMMC1) || defined(SDMMC2)
    #if defined(SDMMC1)
        #define SD_INSTANCE              SDMMC1
    #else
        #define SD_INSTANCE              SDMMC2
    #endif
    #define SD_CLK_EDGE              SDMMC_CLOCK_EDGE_RISING
    #if defined(SDMMC_CLOCK_BYPASS_DISABLE)
            #define SD_CLK_BYPASS            SDMMC_CLOCK_BYPASS_DISABLE
    #endif
    #define SD_CLK_PWR_SAVE          SDMMC_CLOCK_POWER_SAVE_DISABLE
    #define SD_BUS_WIDE_1B           SDMMC_BUS_WIDE_1B
    #define SD_BUS_WIDE_4B           SDMMC_BUS_WIDE_4B
    #define SD_HW_FLOW_CTRL_ENABLE   SDMMC_HARDWARE_FLOW_CONTROL_ENABLE
    #define SD_HW_FLOW_CTRL_DISABLE  SDMMC_HARDWARE_FLOW_CONTROL_DISABLE
    
    #ifndef SD_CLK_DIV
        #if defined(SDMMC_TRANSFER_CLK_DIV)
            #define SD_CLK_DIV               SDMMC_TRANSFER_CLK_DIV
        #else
            #define SD_CLK_DIV               SDMMC_NSpeed_CLK_DIV
        #endif
    #endif

    #if defined(USE_SD_TRANSCEIVER) && (USE_SD_TRANSCEIVER != 0U)
        #if defined(SDMMC_TRANSCEIVER_ENABLE)
            #define SD_TRANSCEIVER_ENABLE    SDMMC_TRANSCEIVER_ENABLE
            #define SD_TRANSCEIVER_DISABLE   SDMMC_TRANSCEIVER_DISABLE
        #else
            #define SD_TRANSCEIVER_ENABLE    SDMMC_TRANSCEIVER_PRESENT
            #define SD_TRANSCEIVER_DISABLE   SDMMC_TRANSCEIVER_NOT_PRESENT
        #endif
    #endif
#elif defined(SDIO)
  #define SD_INSTANCE              SDIO
  #define SD_CLK_EDGE              SDIO_CLOCK_EDGE_RISING
  #if defined(SDIO_CLOCK_BYPASS_DISABLE)
    #define SD_CLK_BYPASS            SDIO_CLOCK_BYPASS_DISABLE
  #endif
  #define SD_CLK_PWR_SAVE          SDIO_CLOCK_POWER_SAVE_DISABLE
  #define SD_BUS_WIDE_1B           SDIO_BUS_WIDE_1B
  #define SD_BUS_WIDE_4B           SDIO_BUS_WIDE_4B
  #define SD_HW_FLOW_CTRL_ENABLE   SDIO_HARDWARE_FLOW_CONTROL_ENABLE
  #define SD_HW_FLOW_CTRL_DISABLE  SDIO_HARDWARE_FLOW_CONTROL_DISABLE
  #ifndef SD_CLK_DIV
    #define SD_CLK_DIV               SDIO_TRANSFER_CLK_DIV
  #endif
#else
  #error "Unknown SD_INSTANCE"
#endif

#ifndef SD_HW_FLOW_CTRL
  #define SD_HW_FLOW_CTRL          SD_HW_FLOW_CTRL_DISABLE
#endif

#ifndef SD_BUS_WIDE
  #define SD_BUS_WIDE              SD_BUS_WIDE_4B
#endif


SDMMCBlockDevice::SDMMCBlockDevice(PinName detect_pin) {
		lock();
   _is_initialized = false;
   _capacity_in_blocks = 0;
   _erase_size = 0;
   _block_size = 0;
   _init_ref_count = 0;
   _cd = detect_pin;
    memset(&_sd_handle, 0, sizeof(_sd_handle));
		unlock();
	  //_sd_handle.Instance = SDMMC1;
         
}

SDMMCBlockDevice::~SDMMCBlockDevice()
{
    if (_is_initialized) {
        deinit();
    }
}

int SDMMCBlockDevice::init()
{
    if (_is_initialized) {
        return BD_ERROR_OK;
    }
		lock();

    int sd_state = BD_ERROR_OK;
    // Initialize pins
  /* Check if SD is not yet initialized */
    if (_sd_handle.State == HAL_SD_STATE_RESET) {
        /* uSD device interface configuration */
        _sd_handle.Instance                 = SD_INSTANCE;
        _sd_handle.Init.ClockEdge           = SD_CLK_EDGE;
#if defined(SD_CLK_BYPASS)
        _sd_handle.Init.ClockBypass         = SD_CLK_BYPASS;
#endif
        _sd_handle.Init.ClockPowerSave      = SD_CLK_PWR_SAVE;
        _sd_handle.Init.BusWide             = SD_BUS_WIDE_1B;
        _sd_handle.Init.HardwareFlowControl = SD_HW_FLOW_CTRL;
        _sd_handle.Init.ClockDiv            = SD_CLK_DIV;
        /* Msp SD initialization */

        //_sd_handle.Init.ClockDiv = _calculate_clock_divider(_freq);
        // Initialize SD interface
        if (HAL_SD_Init(&_sd_handle) != HAL_OK) {
						unlock();
            return BD_ERROR_DEVICE_ERROR;
        }
        /* Configure SD Bus width */
        if (HAL_SD_ConfigWideBusOperation(&_sd_handle, SD_BUS_WIDE) == HAL_OK) {
            sd_state = BD_ERROR_OK;
        } else {
						unlock();
            return BD_ERROR_PARAMETER;
        }
        // Get card info
        if (HAL_SD_GetCardInfo(&_sd_handle, &_card_info) != HAL_OK) {
						unlock();
            return BD_ERROR_DEVICE_ERROR;
        }
        _block_size = _card_info.BlockSize;
				_erase_size = _block_size;
        _capacity_in_blocks = _card_info.BlockNbr;
        _is_initialized = true;
        // Store actual frequency
        _freq = (uint64_t)HAL_RCCEx_GetPeriphCLKFreq(SD_PERIPHCLK) / 
                (2U * (_sd_handle.Init.ClockDiv + 1U));        
    }
		unlock();
    return sd_state;
}

int SDMMCBlockDevice::deinit()
{
		lock();
    if (!_is_initialized) {
        return BD_ERROR_OK;
    }

		_deinit_sdmmc_pins(&_sd_handle);
    if (HAL_SD_DeInit(&_sd_handle) != HAL_OK) {
        return BD_ERROR_DEVICE_ERROR;
    }

    _is_initialized = false;
		unlock();
    return BD_ERROR_OK;
}

int SDMMCBlockDevice::read(void *buffer, bd_addr_t addr, bd_size_t size)
{
    if (!_is_initialized) {
        return BD_ERROR_DEVICE_ERROR;
    }

    if (buffer == nullptr) {
        return BD_ERROR_DEVICE_ERROR;
    }

    if (!is_valid_read(addr, size)) {
        return BD_ERROR_DEVICE_ERROR;
    }
		lock();

    uint32_t block_addr = addr / _block_size;
    uint32_t block_count = size / _block_size;

    HAL_StatusTypeDef status = HAL_SD_ReadBlocks(&_sd_handle,
                                                (uint8_t *)buffer,
                                                block_addr,
                                                block_count,
                                                SD_TIMEOUT);
    
    unlock();
    return status;
}

int SDMMCBlockDevice::program(const void *buffer, bd_addr_t addr, bd_size_t size)
{
    if (!is_valid_program(addr, size)) {
        return BD_ERROR_PARAMETER;
    }
    
		if (buffer == nullptr) {
        return BD_ERROR_DEVICE_ERROR;
    }

    if (!_is_initialized) {
        return BD_ERROR_DEVICE_ERROR;
    }
    lock();

    uint32_t block_addr = addr / _block_size;
    uint32_t block_count = size / _block_size;

    HAL_StatusTypeDef status = HAL_SD_WriteBlocks(&_sd_handle,
                                                 const_cast<uint8_t*>(static_cast<const uint8_t*>(buffer)),
                                                 block_addr,
                                                 block_count,
                                                 SD_TIMEOUT);

    uint32_t timeout = HAL_GetTick() + SD_TIMEOUT;
    while (BSP_SD_GetCardState() != SD_TRANSFER_OK) {
        if (HAL_GetTick() >= timeout) {
            return BD_ERROR_TIMEOUT;
        }
    }

    unlock();
    return BD_ERROR_OK;
}



bool SDMMCBlockDevice::is_valid_write(bd_addr_t addr, bd_size_t size) {
		return ( addr % get_program_size() == 0 &&
						 size % get_program_size() == 0 &&
             addr + size <= this->size());
		// Not checking large reads;
}

bool SDMMCBlockDevice::is_valid_read(bd_addr_t addr, bd_size_t size) {
		return ( addr % get_read_size() == 0 &&
						 size % get_read_size() == 0 &&
             addr + size <= this->size());
		// Not checking large reads;
}

bool SDMMCBlockDevice::is_valid_trim(bd_addr_t addr, bd_size_t size)
{
    return (
               addr % _erase_size == 0 &&
               size % _erase_size == 0 &&
               addr + size <= this->size());
}

int SDMMCBlockDevice::erase(bd_addr_t addr, bd_size_t size)
{
	return 0;
}

bd_size_t SDMMCBlockDevice::get_read_size() const 
{
    return _block_size;
}

bd_size_t SDMMCBlockDevice::get_program_size() const 
{
    return _block_size;
}
const char* SDMMCBlockDevice::get_type() const {

    switch (CARD_SDSC) {
        case CARD_SDSC:
            return "CARD_SDSC";
        case SDCARD_V2HC:
            return "SDCARD_V2HC";
        case SDCARD_SECURE:
            return "SDCARD_SECURE";
        default:
            return "SDCARD_NONE";
    }
}

int SDMMCBlockDevice::get_erase_value() const {
	// Not done
  return _block_size;
}

bd_size_t SDMMCBlockDevice::get_erase_size(bd_addr_t addr) const {
		return _block_size;
}
bd_size_t SDMMCBlockDevice::get_erase_size() const 
{
    return _block_size;
}

bd_size_t SDMMCBlockDevice::size() const 
{
    return _block_size * _capacity_in_blocks;
}

SDMMCBlockDevice::CardType SDMMCBlockDevice::card_type() 
{
    if (!_is_initialized) {
        return SDCARD_NONE;
    }

    switch (_card_info.CardType) {
        case CARD_SDSC:
            return (_card_info.CardVersion == CARD_V1_X) ? SDCARD_V1 : SDCARD_V2;
        case CARD_SDHC_SDXC:
            return SDCARD_V2HC;
        case CARD_SECURED:
            return SDCARD_SECURE;
        default:
            return SDCARD_NONE;
    }
}

int SDMMCBlockDevice::_convert_error(HAL_StatusTypeDef status)
{
    switch (status) {
        case HAL_OK:
            return BD_ERROR_OK;
        case HAL_ERROR:
            return BD_ERROR_DEVICE_ERROR;
        case HAL_BUSY:
            return BD_ERROR_DEVICE_ERROR;
        case HAL_TIMEOUT:
            return BD_ERROR_DEVICE_ERROR;
        default:
            return BD_ERROR_DEVICE_ERROR;
    }
}

bool SDMMCBlockDevice::is_present() 
{
    if (_cd == NC)
        return true;
    DigitalIn card_detect(_cd);
    return card_detect.read() == 0;  // CD pin is active low
}


// Add this to deinit function
void SDMMCBlockDevice::_deinit_sdmmc_pins(SD_HandleTypeDef *hsd)
{
    // Not doing anything here yet
}


// SDMMCBlockDevice.cpp
int SDMMCBlockDevice::trim(bd_addr_t addr, bd_size_t size)
{

    if (!_is_initialized) {
        unlock();
        return BD_ERROR_DEVICE_ERROR;
    }
    
		if (!is_valid_trim(addr, size)) {
        return BD_ERROR_PARAMETER;
    }
    
    lock();

		uint32_t end_block = (addr + size - 1) / _block_size;
    addr = addr / _block_size;

    // For SDSC cards (Standard Capacity), the argument should be in bytes
    // For SDHC/SDXC cards (High/Extended Capacity), the argument should be in blocks
    // The HAL drivers handle this conversion internally based on card type
    HAL_StatusTypeDef status = HAL_SD_Erase(&_sd_handle, addr, end_block);

    // Wait for the card to complete the erase operation
    uint32_t timeout = HAL_GetTick() + SD_TIMEOUT;
    while (BSP_SD_GetCardState() != SD_TRANSFER_OK) {
        if (HAL_GetTick() >= timeout) {
            return BD_ERROR_TIMEOUT;
        }
    }

    return BD_ERROR_OK;
}

/**
 * @brief  Gets the current SD card state.
 * @retval SD card state
 */
uint8_t SDMMCBlockDevice::BSP_SD_GetCardState(void)
{
    HAL_SD_CardStateTypeDef card_state;
    card_state = HAL_SD_GetCardState(&_sd_handle);

    if (card_state == HAL_SD_CARD_TRANSFER) {
        return SD_TRANSFER_OK;
    } else if (card_state == HAL_SD_CARD_ERROR) {
        return SD_TRANSFER_ERROR;
    } else {
        return SD_TRANSFER_BUSY;
    }
}

// SDMMCBlockDevice.cpp
int SDMMCBlockDevice::frequency(uint64_t freq)
{
    if (!_is_initialized) {
        _freq = freq;  // Store for later use in init()
        return BD_ERROR_OK;
    }

    // Calculate appropriate clock divider
    uint32_t clk_div = _calculate_clock_divider(freq);
    
    // Store current state
    uint32_t old_clk_div = _sd_handle.Init.ClockDiv;
    
    // Disable SDMMC peripheral
    HAL_SD_DeInit(&_sd_handle);
    
    // Update clock divider
    _sd_handle.Init.ClockDiv = clk_div;
    
    // Apply new clock configuration
    HAL_StatusTypeDef status = HAL_SD_InitCard(&_sd_handle);
    
    if (status != HAL_OK) {
        // Restore old settings on failure
        _sd_handle.Init.ClockDiv = old_clk_div;
        HAL_SD_InitCard(&_sd_handle);
        return BD_ERROR_DEVICE_ERROR;
    }
    
    // Store actual frequency
    _freq = (uint64_t)HAL_RCCEx_GetPeriphCLKFreq(SD_PERIPHCLK) / (2U * (clk_div + 1U));
    
    return BD_ERROR_OK;
}

uint64_t SDMMCBlockDevice::frequency() const
{
    return _freq;
}

uint32_t SDMMCBlockDevice::_calculate_clock_divider(uint64_t freq)
{
    // Get SDMMC kernel clock
    uint32_t sdmmc_ker_ck = HAL_RCCEx_GetPeriphCLKFreq(SD_PERIPHCLK);
    
    // SDMMC actual clock = SDMMC_CK / (2 * CLKDIV)
    // CLKDIV = (SDMMC_CK / (2 * target_freq)) - 1
    
    uint32_t div;
    if (freq >= sdmmc_ker_ck / 2) {
        // Maximum frequency, minimum divider
        div = 0;
    } else {
        div = (sdmmc_ker_ck / (2 * freq)) - 1;
        
        // STM32H7 SDMMC divider is 8-bit
        if (div > 255) {
            div = 255;  // Maximum divider
        }
    }
    
    return div;
}

void SDMMCBlockDevice::debug(bool dbg) {
   dbg = false; 
}

// SDMMCBlockDevice.cpp
int SDMMCBlockDevice::sync()
{
    if (!_is_initialized) {
        return BD_ERROR_NO_INIT;
    }

    // Wait for any ongoing operations to complete
    uint32_t start = HAL_GetTick();
    HAL_SD_CardStateTypeDef card_state;

    do {
        // Get current card state
        card_state = HAL_SD_GetCardState(&_sd_handle);

        // Check for errors
        if (card_state == HAL_SD_CARD_ERROR) {
            return BD_ERROR_DEVICE_ERROR;
        }

        // Check for timeout
        if ((HAL_GetTick() - start) > SD_TIMEOUT) {
            return BD_ERROR_TIMEOUT;
        }

        // Add a small delay to prevent tight polling
        HAL_Delay(1);

    } while (card_state != HAL_SD_CARD_TRANSFER);

    // Once in transfer state, ensure the data is flushed
#if defined SDIO_FLAG_RXACT
    if (_sd_handle.Instance->STA & (SDIO_FLAG_RXACT | FLAG_CMDACT)) {
        // Wait for any ongoing data or command operations
        uint32_t timeout = HAL_GetTick() + SD_TIMEOUT;
        
        while (_sd_handle.Instance->STA & (SDIO_FLAG_RXACT | FLAG_CMDACT)) {
            if (HAL_GetTick() >= timeout) {
                return BD_ERROR_TIMEOUT;
            }
        }
    }
#endif

    // For cards that support it, issue a FLUSH_CACHE command (CMD32)
#if defined(SDMMC_CCCC_ERASE)
    if (_card_info.Class & SDMMC_CCCC_ERASE) {  // Check if card supports switch commands
        // Create command configuration for flush cache
        SDMMC_CmdInitTypeDef cmd_config = {0};
        cmd_config.Argument = 0;
        cmd_config.CmdIndex = SD_CMD_FLUSH_CACHE;
        cmd_config.Response = SDMMC_RESPONSE_SHORT;
        cmd_config.WaitForInterrupt = SDMMC_WAIT_NO;
        cmd_config.CPSM = SDMMC_CPSM_ENABLE;

        // Send flush cache command
        if (SDMMC_SendCommand(SDMMC1, &cmd_config) == HAL_OK) {
            // Wait for command completion
            uint32_t timeout = HAL_GetTick() + SD_TIMEOUT;
            while (_sd_handle.Instance->STA & FLAG_CMDACT) {
                if (HAL_GetTick() >= timeout) {
                    return BD_ERROR_TIMEOUT;
                }
            }
        } // Probably not supported ignore
    }
#endif

    return BD_ERROR_OK;
}

/**
  * @brief  Initializes the SDIO MSP.
  * @param  hsdio: SDIO handle
  * @retval None
*/
void HAL_SD_MspInit(SD_HandleTypeDef *hsd) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
		/* Enable SDIO clock */
		__HAL_RCC_SDMMC1_CLK_ENABLE();
		
		/* Enable GPIOs clocks */
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();

		/* Common GPIO configuration */
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

		/* STM32 family specific configuration */
		GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		
		GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* NVIC configuration for SDIO interrupts */
		HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd) {

        /* Disable SDIO clock */
        __HAL_RCC_SDMMC1_CLK_DISABLE();

        /* DeInit GPIO pins */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

        /* Disable NVIC for SDIO interrupts */
        HAL_NVIC_DisableIRQ(SDMMC1_IRQn);

}
