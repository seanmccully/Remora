#include "rcservo.h"

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON
************************************************************************/

unique_ptr<Module> createRCServo(const JsonObject& config) {
    const char* comment = config["Comment"];
    int sp = config["SP[i]"];
    const char* pin = config["Servo Pin"];

    // slow module with 10 hz update
    int updateHz = 10;
    return make_unique<RCServo>(&rxData->setPoint[sp], pin, PRU_BASEFREQ, updateHz);
}

/***********************************************************************
*                METHOD DEFINITIONS                                    *
************************************************************************/


RCServo::RCServo(volatile float* ptrPositionCmd, const char* pin, int32_t threadFreq, int32_t slowUpdateFreq) :
	Module(threadFreq, slowUpdateFreq),
	ptrPositionCmd(ptrPositionCmd),
	pin(pin),
	threadFreq(threadFreq)
{

	this->servoPin = new Pin(this->pin, GPIO_MODE_OUTPUT_PP);			// create Pin

	this->T_ms = 20; 	// 50hz
	this->T_compare = this->T_ms * this->threadFreq / 1000;
	this->pinState = false;
	this->counter = 0;
	this->positionCommand = 0;

	this->t_compare = (this->threadFreq / 1000)*(1 + (int)this->positionCommand / 180);
}

void RCServo::update()
{
	counter++;

	if (counter == this->t_compare)
	{
		pinState = false;		// falling edge of pulse
	}
	else if (counter == this->T_compare)
	{
		pinState = true; 		// rising edge of pulse
		counter = 0;
	}

	this->servoPin->set(pinState);
}

void RCServo::slowUpdate()
{
	// the slowUpate is used to update the position set-point

	this->positionCommand = *(this->ptrPositionCmd);
	int t = this->threadFreq*(180 + (int)this->positionCommand)/(1000*180);
	this->t_compare = (int)t;
}
