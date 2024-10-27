#include "debugPin.h"


Debug::Debug(const char* portAndPin, bool bstate) :
    bState(bstate)
{
	this->debugPin = new Pin(portAndPin, OUTPUT);
}

void Debug::update(void)
{
	this->debugPin->set(bState);
}

void Debug::slowUpdate(void)
{
	return;
}
