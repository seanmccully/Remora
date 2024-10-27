#include "eStop.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

unique_ptr<Module> createEStop(const JsonObject& config) {

    const char* comment = config["Comment"];

    const char* pin = config["Pin"];

    //ptrTxHeader = &txData.header;

    return make_unique<eStop>(txData.header, pin);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

eStop::eStop(volatile int32_t &ptrTxHeader, const char* portAndPin) :
    ptrTxHeader(&ptrTxHeader),
	portAndPin(portAndPin)
{
	this->pin = new Pin(this->portAndPin, 0);		// Input 0x0, Output 0x1
}


void eStop::update()
{
    if (this->pin->get() == 1)
    {
        *ptrTxHeader = PRU_ESTOP;
    }
    else {
        *ptrTxHeader = PRU_DATA;
    }
}

void eStop::slowUpdate()
{
	return;
}
