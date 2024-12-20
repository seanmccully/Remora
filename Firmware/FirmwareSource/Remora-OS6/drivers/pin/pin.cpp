#include "mbed.h"

#include "pin.h"


Pin::Pin(const char* portAndPin, int dir) :
    portAndPin(portAndPin),
    dir(dir)
{
    this->modifier = PULLNONE;
    setDir();
    this->configPin();
}

Pin::Pin(const char* portAndPin, int dir, int modifier) :
    portAndPin(portAndPin),
    dir(dir),
    modifier(modifier)
{
    setDir();
    this->configPin();
}

GPIO_TypeDef* Pin::getBus() {
	return this->GPIOx;
}

uint32_t Pin::getPin(void) {
	return this->pin;
}

void Pin::setDir() {
    // Set direction
    if (this->dir == INPUT)
    {
        this->mode = GPIO_MODE_INPUT;

        // Set pin  modifier
        switch(this->modifier)
        {
            case PULLUP:
                this->pull = GPIO_PULLUP;
                break;
            case PULLDOWN:
                this->pull = GPIO_PULLDOWN;
                break;
            case NONE:
            case PULLNONE:
                this->pull = GPIO_NOPULL;
                break;
        }
    }
    else
    {
        this->mode = GPIO_MODE_OUTPUT_PP;
        this->pull = GPIO_NOPULL;
    }
}

void Pin::configPin()
{
    //x can be (A..H) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
    GPIO_TypeDef* gpios[8] ={GPIOA,
#if defined(GPIOB)
                             GPIOB,
#else
                             nullptr,
#endif
#if defined(GPIOC)
                             GPIOC,
#else
                             nullptr,
#endif
#if defined(GPIOD)
                             GPIOD,
#else
                             nullptr,
#endif
#if defined(GPIOE)
                             GPIOE,
#else
                             nullptr,
#endif
#if defined(GPIOF)
                             GPIOF,
#else
                             nullptr,
#endif
#if defined(GPIOG)
                             GPIOG,
#else
                             nullptr,
#endif
#if defined(GPIOH)
                             GPIOH};
#else
                             nullptr};
#endif


    if (portAndPin[0] == 'P') // PXXX e.g.PA2 PC15
    {
        portIndex            = portAndPin[1] - 'A';
        pinNumber            = portAndPin[3] - '0';
        uint32_t pin2        =  portAndPin[4] - '0';

        if (pin2 <= 8)
        {
            this->pinNumber = this->pinNumber * 10 + pin2;
        }

        this->pin = 1 << this->pinNumber; // this is equivalent to GPIO_PIN_x definition
    }
    else
    {
        return;
    }

    // translate port index into something useful
    this->GPIOx = gpios[this->portIndex];

    // enable the peripheral clock
    switch (portIndex){
        case 0:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            break;

        case 1:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            break;

        case 2:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            break;

        case 3:
            __HAL_RCC_GPIOD_CLK_ENABLE();
            break;

        case 4:
            __HAL_RCC_GPIOE_CLK_ENABLE();
            break;

        case 5:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            break;

        case 6:
            __HAL_RCC_GPIOG_CLK_ENABLE();
            break;

        case 7:
            __HAL_RCC_GPIOH_CLK_ENABLE();
            break;
    }

    this->initPin();
}


void Pin::initPin()
{
    // Configure GPIO pin Output Level
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);

    // Configure the GPIO pin
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Pin::setAsOutput()
{
    mode = GPIO_MODE_OUTPUT_PP;
    pull = GPIO_NOPULL;
    initPin();
}


void Pin::setAsInput()
{
    mode = GPIO_MODE_INPUT;
    pull = GPIO_NOPULL;
    initPin();
}


void Pin::pull_none()
{
    pull = GPIO_NOPULL;
    initPin();
}


void Pin::pull_up()
{
    pull = GPIO_PULLUP;
    initPin();
}


void Pin::pull_down()
{
    pull = GPIO_PULLDOWN;
    initPin();
}


PinName Pin::pinToPinName()
{
    return static_cast<PinName>((portIndex << 4) | pinNumber);
}


// If available on this pin, return mbed hardware pwm class for this pin
PwmOut* Pin::hardware_pwm()
{
    if (this->portIndex == 0)
    {
        if (pinNumber == 0) { return new mbed::PwmOut(PA_0); }
        if (pinNumber == 1) { return new mbed::PwmOut(PA_1); }
        if (pinNumber == 2) { return new mbed::PwmOut(PA_2); }
        if (pinNumber == 3) { return new mbed::PwmOut(PA_3); }
        if (pinNumber == 5) { return new mbed::PwmOut(PA_5); }
        if (pinNumber == 6) { return new mbed::PwmOut(PA_6); }
        if (pinNumber == 7) { return new mbed::PwmOut(PA_7); }
        if (pinNumber == 8) { return new mbed::PwmOut(PA_8); }
        if (pinNumber == 9) { return new mbed::PwmOut(PA_9); }
        if (pinNumber == 10) { return new mbed::PwmOut(PA_10); }
        if (pinNumber == 11) { return new mbed::PwmOut(PA_11); }
        if (pinNumber == 15) { return new mbed::PwmOut(PA_15); }
    }
    else if (this->portIndex == 1)
    {
        if (pinNumber == 0) { return new mbed::PwmOut(PB_0); }
        if (pinNumber == 1) { return new mbed::PwmOut(PB_1); }
        if (pinNumber == 3) { return new mbed::PwmOut(PB_3); }
        if (pinNumber == 4) { return new mbed::PwmOut(PB_4); }
        if (pinNumber == 5) { return new mbed::PwmOut(PB_5); }
        if (pinNumber == 6) { return new mbed::PwmOut(PB_6); }
        if (pinNumber == 7) { return new mbed::PwmOut(PB_7); }
        if (pinNumber == 8) { return new mbed::PwmOut(PB_8); }
        if (pinNumber == 9) { return new mbed::PwmOut(PB_9); }
        if (pinNumber == 10) { return new mbed::PwmOut(PB_10); }
        if (pinNumber == 11) { return new mbed::PwmOut(PB_11); }
        if (pinNumber == 13) { return new mbed::PwmOut(PB_13); }
        if (pinNumber == 14) { return new mbed::PwmOut(PB_14); }
        if (pinNumber == 15) { return new mbed::PwmOut(PB_15); }
    }
    else if (this->portIndex == 2)
    {
        if (pinNumber == 6) { return new mbed::PwmOut(PC_6); }
        if (pinNumber == 7) { return new mbed::PwmOut(PC_7); }
        if (pinNumber == 8) { return new mbed::PwmOut(PC_8); }
        if (pinNumber == 9) { return new mbed::PwmOut(PC_9); }
    }
    else if (this->portIndex == 3)
    {
        if (pinNumber == 12) { return new mbed::PwmOut(PD_12); }
        if (pinNumber == 13) { return new mbed::PwmOut(PD_13); }
        if (pinNumber == 14) { return new mbed::PwmOut(PD_14); }
        if (pinNumber == 15) { return new mbed::PwmOut(PD_15); }
    }
    else if (this->portIndex == 4)
    {
        if (pinNumber == 5) { return new mbed::PwmOut(PE_5); }
        if (pinNumber == 6) { return new mbed::PwmOut(PE_6); }
        if (pinNumber == 8) { return new mbed::PwmOut(PE_8); }
        if (pinNumber == 9) { return new mbed::PwmOut(PE_9); }
        if (pinNumber == 10) { return new mbed::PwmOut(PE_10); }
        if (pinNumber == 11) { return new mbed::PwmOut(PE_11); }
        if (pinNumber == 12) { return new mbed::PwmOut(PE_12); }
        if (pinNumber == 13) { return new mbed::PwmOut(PE_13); }
        if (pinNumber == 14) { return new mbed::PwmOut(PE_14); }
    }
    return nullptr;
}
