#include "mbed.h"

#include "pin.h"

Pin::Pin(uint32_t gpio, uint32_t mode) :
    pinName( static_cast<PinName> ( gpio ) ),
    gpioNumber( ( 1 << ( gpio % 16 ) ) ),
    GPIOx( getGPIOx( (uint8_t) (gpio/16) ) ),
    mode( mode ) {
        gpioSetup();
    }

Pin::Pin(const char* portAndPin, uint32_t mode) : pinName( strToPinName(portAndPin) ), mode(mode) {
    gpioNumber = 1 << ((uint32_t)pinName % 16);
    gpioSetup();
}

Pin::Pin(const char* portAndPin, uint32_t mode, uint32_t pull, bool invert) :
    pinName( strToPinName(portAndPin) ),
    mode( mode )
    {
        gpioNumber = 1 << ((uint32_t)pinName % 16);
        if (!ovrrde) {
            this->invert = invert;
            this->pull = pull;
        }
        gpioSetup();
    }

Pin::Pin(const char* portAndPin, uint32_t mode, uint32_t pull, bool invert, uint32_t altFunc) :
    pinName( strToPinName(portAndPin) ),
    mode( mode | ( altFunc << 4 ) )
 {
    gpioNumber = 1 << ((uint32_t)pinName % 16);
	if (!ovrrde) {
		this->invert = invert;
		this->pull = pull;
	}
    gpioSetup();
 }

Pin::Pin(uint32_t gpio, uint32_t mode, uint32_t pull, bool invert, uint32_t altFunc) :
    pinName( static_cast<PinName> ( gpio ) ),
    mode( mode | ( altFunc << 4 ) ),
    gpioNumber( ( 1 << ( gpio % 16 ) ) ),
    GPIOx( getGPIOx( (uint8_t) (gpio/16) ) )
{
	if (!ovrrde) {
		this->invert = invert;
		this->pull = pull;
	}
    gpioSetup();
}


void Pin::setPinState() {
    setPin(invert);
}

void Pin::resetPinState() {
    setPin(!invert);
}

GPIO_TypeDef* Pin::getBus() {
	return GPIOx;
}

uint32_t Pin::getGPIONumber(void) {
	return gpioNumber;
}

uint8_t Pin::readPin(void) {
    return !!(GPIOx->IDR & gpioNumber);
    //return HAL_GPIO_ReadPin(GPIOx, gpioNumber) == GPIO_PIN_SET;
}

PinName Pin::strToPinName(const char* gpio_str) {
	uint8_t l = strlen(gpio_str), strt = 0, max_l = 5;
    char* upperStr = (char*)malloc(l + 1);
	strcpy(upperStr, gpio_str);
	ovrrde = false;
	for (char* p = upperStr; *p; ++p) {
        *p = std::toupper(*p);
    }

    bool invert   = false;
    uint32_t pull = GPIO_NOPULL;

    // Check for preprocessor symbols
	if (upperStr[strt] == '!') {
		invert = true;
		ovrrde = true;
		strt++;
		max_l++;
	}
	if (upperStr[strt] == '^') {
		pull = GPIO_PULLUP;
		ovrrde = true;
		strt++;
		max_l++;
	} else if (upperStr[strt] == '~') {
		pull = GPIO_PULLDOWN;
		ovrrde = true;
		strt++;
		max_l++;
	}

    // Check format starts with 'P'
    if (upperStr[strt] != 'P') {
        return NC;
    }
	strt++;

    char port = upperStr[strt];

    // Find pin number - handle both "PD5" and "PD_5" formats
    size_t numStart = ++strt;
	if (upperStr[numStart] == '_')
		numStart++;

    if (numStart >= l)
        return NC;

    // Convert pin number string to integer
    const char* numStr = upperStr + numStart;
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
        case 'A': GPIOx = getGPIOx(0); portNum = 0; break;
        case 'B': GPIOx = getGPIOx(1); portNum = 1; break;
        case 'C': GPIOx = getGPIOx(2); portNum = 2; break;
        case 'D': GPIOx = getGPIOx(3); portNum = 3; break;
        case 'E': GPIOx = getGPIOx(4); portNum = 4; break;
        case 'F': GPIOx = getGPIOx(5); portNum = 5; break;
        case 'G': GPIOx = getGPIOx(6); portNum = 6; break;
        case 'H': GPIOx = getGPIOx(7); portNum = 7; break;
        case 'I': GPIOx = getGPIOx(8); portNum = 8; break;
        case 'J': GPIOx = getGPIOx(9); portNum = 9; break;
        default: return NC;
    }

    // Set pin configurations
	if (ovrrde) {
    	this->invert = invert;
    	this->pull = pull;
	}
    // Calculate pin value: (port << 4) | pin
    return static_cast<PinName>((portNum << 4) | pinNum);
}

GPIO_TypeDef* Pin::getGPIOx(int idx) {

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

    // translate port index into something useful
    return gpios[idx];
}
void Pin::setGPIOx() {
    this->GPIOx = getGPIOx((uint8_t) (pinName/16));

}

void Pin::clkEnable() {

    uint8_t portIndex = (uint8_t)(pinName/16);
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
}

void Pin::setFastSpeed() {
    _ospeed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioReset();
}

void Pin::setHighSpeed() {
    _ospeed = GPIO_SPEED_FREQ_HIGH;
    gpioReset();
}

void Pin::setMediumSpeed() {
    _ospeed = GPIO_SPEED_FREQ_MEDIUM;
    gpioReset();
}

void Pin::setLowSpeed() {
    _ospeed = GPIO_SPEED_FREQ_MEDIUM;
    gpioReset();
}

void Pin::setAsOutput()
{
    mode = GPIO_MODE_OUTPUT_PP;
    pull = GPIO_NOPULL;
    gpioReset();
}


void Pin::setAsInput()
{
    mode = GPIO_MODE_INPUT;
    pull = GPIO_NOPULL;
    gpioReset();
}

void Pin::pullNone()
{
    pull = GPIO_NOPULL;
    gpioReset();
}


void Pin::pullUp()
{
    pull = GPIO_PULLUP;
    gpioReset();
}


void Pin::pullDown()
{
    pull = GPIO_PULLDOWN;
    gpioReset();
}


PinName Pin::pinToPinName()
{
    return static_cast<PinName>(gpioNumber);
}

void Pin::gpioSetup() {
    gpioClockEnable();
    clkEnable();
    gpioReset();
}

void Pin::gpioReset() {

    irqstatus_t flag = irq_save();

    if (mode == GPIO_MODE_OUTPUT_PP)
        setPin( invert );

    gpioPeripheral();

    irq_restore(flag);
}

void Pin::togglePin() {
    irqstatus_t flag = irq_save();
    GPIOx->ODR ^= gpioNumber;
    irq_restore(flag);
}

void Pin::setPin(bool val) {
    if (val)
        GPIOx->BSRR = gpioNumber;
    else
        GPIOx->BSRR = gpioNumber << 16;
}

#if defined TARGET_STM32H7
// Enable a GPIO peripheral clock
void Pin::gpioClockEnable() {

    uint32_t pos = ((uint32_t)GPIOx - D3_AHB1PERIPH_BASE) / 0x400;
    RCC->AHB4ENR |= (1<<pos);
    RCC->AHB4ENR;
}
#elif defined TARGET_STM32F1
void Pin::gpioClockEnable() {
{
    uint32_t rcc_pos = ((uint32_t)GPIOx - APB2PERIPH_BASE) / 0x400;
    RCC->APB2ENR |= 1 << rcc_pos;
    RCC->APB2ENR;
}
#elif defined TARGET_STM32F4
// Enable a GPIO peripheral clock
void Pin::gpioClockEnable() {
    uint32_t rcc_pos = ((uint32_t)GPIOx - AHB1PERIPH_BASE) / 0x400;
    RCC->AHB1ENR |= 1 << rcc_pos;
    RCC->AHB1ENR;
}
#elif defined TARGET_STM32G0
// Enable a GPIO peripheral clock
void Pin::gpioClockEnable() {

    uint32_t rcc_pos = ((uint32_t)GPIOx - IOPORT_BASE) / 0x400;
    RCC->IOPENR |= 1 << rcc_pos;
    RCC->IOPENR;
}
#endif

void Pin::gpioPeripheral() {
    // Enable GPIO clock

    gpioClockEnable();

    // Configure GPIO
    uint32_t mode_bits = mode & 0xf, func = (mode >> 4) & 0xf;
    uint32_t od = (mode >> 8) & 0x1, hs = (mode >> 9) & 0x1;
    uint32_t pup = pull ? (pull == 1 ? 1 : 2) : 0;
    uint32_t pos = (pinName % 16), af_reg = pos / 8;
    uint32_t af_shift = (pos % 8) * 4, af_msk = 0x0f << af_shift;
    uint32_t m_shift = pos * 2, m_msk = 0x03 << m_shift;

    if ((mode_bits & MODE_AF) == MODE_AF)
        GPIOx->AFR[af_reg] = (GPIOx->AFR[af_reg] & ~af_msk) | (func << af_shift);
    GPIOx->MODER = (GPIOx->MODER & ~m_msk) | (mode_bits << m_shift);
    GPIOx->PUPDR = (GPIOx->PUPDR & ~m_msk) | (pup << m_shift);
    GPIOx->OTYPER = (GPIOx->OTYPER & ~(1 << pos)) | (od << pos);

    // Setup OSPEEDR:
    // stm32f0 is ~10Mhz at 50pF
    // stm32f2 is ~25Mhz at 40pF
    // stm32f4 is ~50Mhz at 40pF
    // stm32f7 is ~50Mhz at 40pF
    // stm32g0 is ~30Mhz at 50pF
    // stm32h7 is ~85Mhz at 50pF
    //uint32_t ospeed = hs ? 0x03 : (0x02); // STM32F0 = 0x01
    uint32_t ospeed = _ospeed;
    GPIOx->OSPEEDR = (GPIOx->OSPEEDR & ~m_msk) | (ospeed << m_shift);
}


// If available on this pin, return mbed hardware pwm class for this pin
PwmOut* Pin::hardware_pwm()
{
    uint8_t portIndex = (uint8_t)(pinName/16);
    uint8_t pinNumber = (uint8_t)(pinName % 16);
    if ((portIndex) == 0)
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
    else if (portIndex == 1)
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
    else if (portIndex == 2)
    {
        if (pinNumber == 6) { return new mbed::PwmOut(PC_6); }
        if (pinNumber == 7) { return new mbed::PwmOut(PC_7); }
        if (pinNumber == 8) { return new mbed::PwmOut(PC_8); }
        if (pinNumber == 9) { return new mbed::PwmOut(PC_9); }
    }
    else if (portIndex == 3)
    {
        if (pinNumber == 12) { return new mbed::PwmOut(PD_12); }
        if (pinNumber == 13) { return new mbed::PwmOut(PD_13); }
        if (pinNumber == 14) { return new mbed::PwmOut(PD_14); }
        if (pinNumber == 15) { return new mbed::PwmOut(PD_15); }
    }
    else if (portIndex == 4)
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
