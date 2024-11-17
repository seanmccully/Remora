

#include "board.h"
//*** Initialization Functions ***
//*** PinMap Structure Reference ***
// typedef struct {
//     PinName pin;
//     int peripheral;
//     int function;
// } PinMap;

//*** Initialization Functions ***
#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_OPEN_DRAIN 0x100
#define GPIO_HIGH_SPEED 0x200
#define GPIO_FUNCTION(fn) (2 | ((fn) << 4))
#define GPIO_ANALOG 3
#define SDIO_FUNCTION GPIO_FUNCTION(12)


MBED_WEAK void init_probe_pins(void) {
    gpio_t gpio;

    // Initialize probe pins based on PinMap_PROBE
    for(int i = 0; PinMap_PROBE[i].pin != NC; i++) {
        if(PinMap_PROBE[i].peripheral == PIN_INPUT) {
            // Configure input pins with pullup
            gpio_init_in(&gpio, PinMap_PROBE[i].pin);
            gpio_mode(&gpio, PullUp);
        } else {
            // Configure PWM pin for BLTouch servo
            pin_function(PinMap_PROBE[i].pin, PinMap_PROBE[i].function);
        }
    }
}

MBED_WEAK void init_spi1(void) {
    gpio_t gpio;

    // Configure SPI1 MOSI/MISO/SCK pins
    for(int i = 0; PinMap_SPI_MOSI[i].pin != NC; i++) {
        if(PinMap_SPI_MOSI[i].peripheral == SPI_1) {
            pin_function(PinMap_SPI_MOSI[i].pin, PinMap_SPI_MOSI[i].function);
        }
    }

    for(int i = 0; PinMap_SPI_MISO[i].pin != NC; i++) {
        if(PinMap_SPI_MISO[i].peripheral == SPI_1) {
            pin_function(PinMap_SPI_MISO[i].pin, PinMap_SPI_MISO[i].function);
        }
    }

    for(int i = 0; PinMap_SPI_SCLK[i].pin != NC; i++) {
        if(PinMap_SPI_SCLK[i].peripheral == SPI_1) {
            pin_function(PinMap_SPI_SCLK[i].pin, PinMap_SPI_SCLK[i].function);
        }
    }

    // Initialize all SPI1 chip select pins
    for(int i = 0; PinMap_SPI_SSEL[i].pin != NC; i++) {
        if(PinMap_SPI_SSEL[i].peripheral == SPI_1) {
            gpio_init_out(&gpio, PinMap_SPI_SSEL[i].pin);
            gpio_write(&gpio, 1);  // Active LOW, start disabled
        }
    }
}

MBED_WEAK void init_spi2(void) {
    gpio_t gpio;

    // Configure SPI2 MOSI/MISO/SCK pins
    for(int i = 0; PinMap_SPI_MOSI[i].pin != NC; i++) {
        if(PinMap_SPI_MOSI[i].peripheral == SPI_2) {
            pin_function(PinMap_SPI_MOSI[i].pin, PinMap_SPI_MOSI[i].function);
        }
    }

    for(int i = 0; PinMap_SPI_MISO[i].pin != NC; i++) {
        if(PinMap_SPI_MISO[i].peripheral == SPI_2) {
            pin_function(PinMap_SPI_MISO[i].pin, PinMap_SPI_MISO[i].function);
        }
    }

    for(int i = 0; PinMap_SPI_SCLK[i].pin != NC; i++) {
        if(PinMap_SPI_SCLK[i].peripheral == SPI_2) {
            pin_function(PinMap_SPI_SCLK[i].pin, PinMap_SPI_SCLK[i].function);
        }
    }

    // Initialize SPI2 chip select
    for(int i = 0; PinMap_SPI_SSEL[i].pin != NC; i++) {
        if(PinMap_SPI_SSEL[i].peripheral == SPI_2) {
            gpio_init_out(&gpio, PinMap_SPI_SSEL[i].pin);
            gpio_write(&gpio, 1);  // Active LOW, start disabled
        }
    }
}

MBED_WEAK void init_spi3(void) {
    gpio_t gpio;

    // Configure SPI3 MOSI/MISO/SCK pins
    for(int i = 0; PinMap_SPI_MOSI[i].pin != NC; i++) {
        if(PinMap_SPI_MOSI[i].peripheral == SPI_3) {
            pin_function(PinMap_SPI_MOSI[i].pin, PinMap_SPI_MOSI[i].function);
        }
    }

    for(int i = 0; PinMap_SPI_MISO[i].pin != NC; i++) {
        if(PinMap_SPI_MISO[i].peripheral == SPI_3) {
            pin_function(PinMap_SPI_MISO[i].pin, PinMap_SPI_MISO[i].function);
        }
    }

    for(int i = 0; PinMap_SPI_SCLK[i].pin != NC; i++) {
        if(PinMap_SPI_SCLK[i].peripheral == SPI_3) {
            pin_function(PinMap_SPI_SCLK[i].pin, PinMap_SPI_SCLK[i].function);
        }
    }

    // Initialize SPI3 chip select
    for(int i = 0; PinMap_SPI_SSEL[i].pin != NC; i++) {
        if(PinMap_SPI_SSEL[i].peripheral == SPI_3) {
            gpio_init_out(&gpio, PinMap_SPI_SSEL[i].pin);
            gpio_write(&gpio, 1);  // Active LOW, start disabled
        }
    }
}

MBED_WEAK void init_board_pins(void) {
    gpio_t gpio;

    // Initialize stepper motor pins
    for(int i = 0; PinMap_STEPPER_STEP[i].pin != NC; i++) {
        gpio_init_out(&gpio, PinMap_STEPPER_STEP[i].pin);
        gpio_write(&gpio, 0);
    }

    for(int i = 0; PinMap_STEPPER_DIR[i].pin != NC; i++) {
        gpio_init_out(&gpio, PinMap_STEPPER_DIR[i].pin);
        gpio_write(&gpio, 0);
    }

    for(int i = 0; PinMap_STEPPER_ENABLE[i].pin != NC; i++) {
        gpio_init_out(&gpio, PinMap_STEPPER_ENABLE[i].pin);
        gpio_write(&gpio, 1);  // Active LOW, disabled by default
    }

    // Initialize endstops with pullups
    for(int i = 0; PinMap_ENDSTOPS[i].pin != NC; i++) {
        gpio_init_in(&gpio, PinMap_ENDSTOPS[i].pin);
        gpio_mode(&gpio, PullUp);
    }

    // Initialize LCD pins
    for(int i = 0; PinMap_LCD[i].pin != NC; i++) {
        gpio_init_out(&gpio, PinMap_LCD[i].pin);
        gpio_write(&gpio, 0);
    }

    // Initialize UI pins
    for(int i = 0; PinMap_UI[i].pin != NC; i++) {
        gpio_init_in(&gpio, PinMap_UI[i].pin);
        gpio_mode(&gpio, PullUp);
    }

    // Disable SPI CS Pins
    for(int i = 0; PinMap_TMC_SPI[i].pin != NC; i++) {
        gpio_init_out(&gpio, PinMap_TMC_SPI[i].pin);
        gpio_write(&gpio, 1);  // Active Low, Disabled by default
    }

    // Initialize other subsystems
    init_probe_pins();

}

MBED_WEAK void init_sdio(void) {
    // Configure SDIO pins with pullups
    for(int i = 0; PinMap_SD[i].pin != NC; i++) {
        pin_function(PinMap_SD[i].pin, PinMap_SD[i].function);
        gpio_t gpio;
        gpio_init_out(&gpio, PinMap_SD[i].pin);
        gpio_mode(&gpio, PullUp);
    }
}


MBED_WEAK void set_sdio_clock(bool enable, PinName sdio_ck) {
    const PinMap *sdio_map = PinMap_SD;

    // Find clock pin
    while(sdio_map->pin != NC) {
        if(sdio_map->pin == sdio_ck) {  // SDIO_CK pin
            if(enable) {
                // Enable SDIO clock - configure pin in AF mode
                pin_function(sdio_map->pin, sdio_map->function|GPIO_HIGH_SPEED);
            } else {
                // Disable SDIO clock - configure pin as input
                gpio_t gpio;
                gpio_init_in(&gpio, sdio_map->pin);
                gpio_mode(&gpio, PullNone);
            }
            break;
        }
        sdio_map++;
    }
}

// Helper function for SDIO card detection if needed
MBED_WEAK bool is_sd_card_present(void) {
    gpio_t gpio;
    bool card_present = true;

    // If there's a card detect pin defined, use it
    #ifdef PIN_DETECT
        if (PIN_DETECT == NC)
            return card_present;
        gpio_init_in(&gpio, PIN_DETECT);
        gpio_mode(&gpio, PullUp);
        card_present = (gpio_read(&gpio) == SD_DETECT_PRESENT_STATE);
    #endif

    return card_present;
}

// SDIO power control if needed
MBED_WEAK void sdio_power_control(bool enable) {
    gpio_t gpio;

    #ifdef SDIO_POWER_PIN
        gpio_init_out(&gpio, SDIO_POWER_PIN);
        gpio_write(&gpio, enable ? SDIO_POWER_ON_STATE : !SDIO_POWER_ON_STATE);
    #endif
}


// Full SDIO initialization sequence
MBED_WEAK int init_sdio_interface(void) {
    // Initialize basic SDIO pins
    init_sdio();

    // Check for card presence if supported
    if (!is_sd_card_present()) {
        return -1;  // No card present
    }

    // Power up sequence
    sdio_power_control(false);  // Power off first
    wait_us(1000);             // Wait 1ms
    sdio_power_control(true);   // Power on
    wait_us(1000);             // Wait for power to stabilize

    return 0;  // Success
}
