#include "resetPin.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

unique_ptr<Module> createResetPin(const JsonObject& config) {
    const char* comment = config["Comment"];
    const char* pin = config["Pin"];

    return  make_unique<ResetPin>(PRUreset, pin);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

ResetPin::ResetPin(volatile bool &ptrReset, const char* portAndPin) :
	ptrReset(&ptrReset),
	portAndPin(portAndPin)
{
	this->pin = new Pin(portAndPin, 0);		// Input 0x0, Output 0x1
}


void ResetPin::update()
{
	*(this->ptrReset) = this->pin->get();
}

void ResetPin::slowUpdate()
{
	return;
}
