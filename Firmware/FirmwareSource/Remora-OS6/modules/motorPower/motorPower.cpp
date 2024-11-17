#include "motorPower.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

unique_ptr<Module> createMotorPower(const JsonObject& config) {
    const char* comment = config["Comment"];
    const char* pin = config["Pin"];

    return make_unique<MotorPower>(pin);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

MotorPower::MotorPower(const char* portAndPin) :
	portAndPin(portAndPin)
{
	this->pin = new Pin(this->portAndPin, 0x1);		// Input 0x0, Output 0x1
    this->update();
}


void MotorPower::update()
{
	this->pin->setPin(true);			// turn motor power ON
}

void MotorPower::slowUpdate()
{
	return;
}
