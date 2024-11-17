#ifndef PIN_H
#define PIN_H

#include "mbed.h"

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <algorithm>
using namespace std;

#if defined TARGET_STM32H7
#include "stm32h7xx_hal.h"
#elif defined TARGET_STM32F1
#include "stm32f1xx_hal.h"
#elif defined TARGET_STM32F4
#include "stm32f4xx_hal.h"
#elif defined TARGET_STM32G0
#include "stm32g0xx_hal.h"
#endif

#include "arm_irq.h"

#define GPIO_INPUT  GPIO_MODE_INPUT
#define GPIO_OUTPUT GPIO_MODE_OUTPUT_PP
#define GPIO_AF     GPIO_MODE_AF_PP

#define NONE        GPIO_NOPULL
#define PULLUP      GPIO_PULLUP
#define PULLDOWN    GPIO_PULLDOWN
#define PULLNONE    GPIO_NOPULL
#define GPIO_ANALOG GPIO_MODE_ANALOG

class Pin
{
    private:

        bool                invert = false;
		bool				ovrrde = false;
        uint8_t             _ospeed = GPIO_SPEED_FREQ_HIGH;
        uint16_t            gpioNumber;
        uint32_t            mode = GPIO_MODE_OUTPUT_PP;
        uint32_t            pull = GPIO_NOPULL;
        uint32_t            speed;
        PinName             pinName;
        GPIO_TypeDef*       GPIOx;

    public:

	    Pin() : gpioNumber(0), mode(GPIO_OUTPUT), pull(GPIO_NOPULL) {} // Default Construct
		Pin(const char*, uint32_t);
        Pin(uint32_t gpio, uint32_t mode);
		Pin(const char* portAndPin, uint32_t mode, uint32_t pull, bool invert);
        Pin(uint32_t, uint32_t mode, uint32_t pull, bool invert, uint32_t altMode);
        Pin(const char*, uint32_t mode,uint32_t pull, bool invert, uint32_t altMode);
        ~Pin() {
            HAL_GPIO_DeInit(GPIOx, gpioNumber);
        }

        PwmOut* hardware_pwm();
        GPIO_TypeDef* getBus(void);
        PinName strToPinName(const char *);
        uint32_t getBit(void);
        uint8_t readPin(void);
        void setPinState(void);
        void resetPinState(void);

        void clkEnable();
        void setAsOutput();
        void setAsInput();
        void pullNone();
        void pullUp();
        void pullDown();
        static GPIO_TypeDef* getGPIOx(int);
        uint32_t getGPIONumber();
        void setGPIOx();
        inline uint32_t getPinN(void) { return (gpioNumber % 16); }
        inline uint32_t getPinB(void) { return 1 << (gpioNumber % 16); }
        inline uint32_t getGPION(void) { return this->gpioNumber; }
        inline uint32_t getMode(void) { return this->mode; }

        void gpioSetup();
        void gpioReset();
        void gpioPeripheral();
        void gpioClockEnable();
        void setPin(bool);
        void togglePin();
		void setFastSpeed();
		void setHighSpeed();
		void setMediumSpeed();
		void setLowSpeed();

        PinName pinToPinName();

        inline bool get()
        {
            return HAL_GPIO_ReadPin(GPIOx, gpioNumber);
        }

        inline void set(bool value)
        {
            if (value)
            {
                HAL_GPIO_WritePin(this->GPIOx, this->gpioNumber, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(this->GPIOx, this->gpioNumber, GPIO_PIN_RESET);
            }
        }
};

#endif
