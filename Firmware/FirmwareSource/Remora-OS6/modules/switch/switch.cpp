#include "switch.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON
************************************************************************/

unique_ptr<Module> createSwitch(const JsonObject& config) {
    const char* comment = config["Comment"];

    const char* pin = config["Pin"];
    const char* mode = config["Mode"];
    int pv = config["PV[i]"];
    float sp = config["SP"];

    if (!strcmp(mode,"On"))
    {
        return make_unique<Switch>(sp, &txData->processVariable[pv], pin, 1);
    }
    else if (!strcmp(mode,"Off"))
    {
        return make_unique<Switch>(sp, &txData->processVariable[pv], pin, 0);
    }
    return nullptr;
}

Switch::Switch(float SP, volatile float* ptrPV, const char* portAndPin, bool mode) :
	SP(SP),
	ptrPV(ptrPV),
	portAndPin(portAndPin),
	mode(mode)
{
	int output = 0x1; // an output
	this->pin = new Pin(this->portAndPin, output);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

void Switch::update()
{
	bool pinState;

	pinState = this->mode;

	// update the SP
	this->PV = *(this->ptrPV);

	if (this->PV > this->SP)
	{
		this->pin->set(pinState);
	}
	else
	{
		pinState = !pinState;
		this->pin->set(pinState);
	}

}


void Switch::slowUpdate()
{
	return;
}
