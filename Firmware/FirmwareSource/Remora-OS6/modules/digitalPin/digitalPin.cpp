#include "digitalPin.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

unique_ptr<Module> createDigitalPin(const JsonObject& config) {
    const char* comment = config["Comment"];

    const char* pin = config["Pin"];
    const char* mode = config["Mode"];
    const char* invert = config["Invert"];
    const char* modifier = config["Modifier"];
    int dataBit = config["Data Bit"];

    int mod;
    bool inv;

    if (!strcmp(modifier,"Open Drain"))
    {
        mod = OPENDRAIN;
    }
    else if (!strcmp(modifier,"Pull Up"))
    {
        mod = PULLUP;
    }
    else if (!strcmp(modifier,"Pull Down"))
    {
        mod = PULLDOWN;
    }
    else if (!strcmp(modifier,"Pull None"))
    {
        mod = PULLNONE;
    }
    else
    {
        mod = NONE;
    }

    if (!strcmp(invert,"True"))
    {
        inv = true;
    }
    else inv = false;


    if (!strcmp(mode,"Output"))
    {
        return std::make_unique<DigitalPin>(ptrRxData->outputs, 1, pin, dataBit, inv, mod);
    }
    else if (!strcmp(mode,"Input"))
    {
        std::make_unique<DigitalPin>(ptrTxData->inputs, 0, pin, dataBit, inv, mod);
    }

    return nullptr;
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

DigitalPin::DigitalPin(volatile uint16_t &ptrData, int mode, const char* portAndPin, int bitNumber, bool invert, int modifier) :
	ptrData(&ptrData),
	mode(mode),
	portAndPin(portAndPin),
	bitNumber(bitNumber),
    invert(invert),
	modifier(modifier)
{
	this->pin = new Pin(this->portAndPin, this->mode, this->modifier);		// Input 0x0, Output 0x1
	this->mask = 1 << this->bitNumber;
}


void DigitalPin::update()
{
	bool pinState;

	if (this->mode == 0)									// the pin is configured as an input
	{
		pinState = this->pin->get();
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
