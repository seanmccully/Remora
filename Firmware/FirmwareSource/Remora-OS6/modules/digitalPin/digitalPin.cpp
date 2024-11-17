#include "digitalPin.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON
************************************************************************/

unique_ptr<Module> createDigitalPin(const JsonObject& config) {
    const char* comment = config["Comment"];

    const char* pin = config["Pin"];
    const char* mode = config["Mode"];
    const char* invert = config["Invert"];
    int dataBit = config["Data Bit"];

    int mod;
    bool inv;


    if (!strcmp(invert,"True"))
    {
        inv = true;
    }
    else inv = false;


    if (!strcmp(mode,"Output"))
    {
        return std::make_unique<DigitalPin>(rxData->outputs, 1, pin, dataBit, inv);
    }
    else if (!strcmp(mode,"Input"))
    {
        std::make_unique<DigitalPin>(txData->inputs, 0, pin, dataBit, inv);
    }

    return nullptr;
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

DigitalPin::DigitalPin(volatile uint16_t &ptrData, int mode, const char* portAndPin, int bitNumber, bool invert) :
	ptrData(&ptrData),
	mode(mode),
	portAndPin(portAndPin),
	bitNumber(bitNumber),
    invert(invert)
{
	this->pin = new Pin(portAndPin, mode);		// Input 0x0, Output 0x1
	this->mask = this->pin->getGPIONumber();
    this->pin->setPinState();
}


void DigitalPin::update()
{
	bool pinState;

	if (this->mode == 0)									// the pin is configured as an input
	{
		pinState = this->pin->readPin();
		if(this->invert)
		{
			pinState = !pinState;
		}

		if (pinState == 1)								// input is high
		{
			*(this->ptrData) |= this->mask;
		}
		else											// input is low
		{
			*(this->ptrData) &= ~this->mask;
		}
	}
	else												// the pin is configured as an output
	{
		pinState = *(this->ptrData) & this->mask;		// get the value of the bit in the data source
		if(this->invert)
		{
			pinState = !pinState;
		}
		this->pin->set(pinState);			// simple conversion to boolean
	}
}

void DigitalPin::slowUpdate()
{
	return;
}
