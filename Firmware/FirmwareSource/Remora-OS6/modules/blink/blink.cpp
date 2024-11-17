#include "blink.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON
************************************************************************/

unique_ptr<Module> createBlink(const JsonObject& config) {
    const char* pin = config["Pin"];
    int frequency = config["Frequency"];

    return make_unique<Blink>(pin, PRU_SERVOFREQ, frequency);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

Blink::Blink(const char* portAndPin, uint32_t threadFreq, uint32_t freq)
{

	this->periodCount = threadFreq / freq;
	this->blinkCount = 0;
	this->bState = false;

	this->blinkPin = new Pin(portAndPin, GPIO_MODE_OUTPUT_PP);
	this->blinkPin->togglePin();
}

void Blink::update(void)
{
	++this->blinkCount;
	if (this->blinkCount >= this->periodCount / 2)
	{
		this->blinkPin->set(this->bState=!this->bState);
		this->blinkCount = 0;
	}
}

void Blink::slowUpdate(void)
{
	return;
}
