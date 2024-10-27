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


#define INPUT 0x0
#define OUTPUT 0x1

#define NONE        0b000
#define OPENDRAIN   0b001
#define PULLUP      0b010
#define PULLDOWN    0b011
#define PULLNONE    0b100

class Pin
{
    private:

        const char*         portAndPin;
        uint8_t             dir;
        uint8_t             modifier;
        uint8_t             portIndex;
        uint16_t            pinNumber;
        uint16_t            pin;
        uint32_t            mode;
        uint32_t            pull;
        uint32_t            speed;
        GPIO_TypeDef*       GPIOx;
        GPIO_InitTypeDef    GPIO_InitStruct = {0};

    public:

        Pin(const char*, int);
        Pin(const char*, int, int);

        PwmOut* hardware_pwm();

        void configPin();
        void initPin();
        void setAsOutput();
        void setAsInput();
        void pull_none();
        void pull_up();
        void pull_down();
        PinName pinToPinName();

        inline bool get()
        {
            return HAL_GPIO_ReadPin(this->GPIOx, this->pin);
        }

        inline void set(bool value)
        {
            if (value)
            {
                HAL_GPIO_WritePin(this->GPIOx, this->pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(this->GPIOx, this->pin, GPIO_PIN_RESET);
            }
        }
};

#endif
