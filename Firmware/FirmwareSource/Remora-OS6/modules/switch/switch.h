#ifndef SWITCH_H
#define SWITCH_H

#include <cstdint>
//#include <iostream>

#include "modules/module.h"
#include "drivers/pin/pin.h"

#include "remora.h"

unique_ptr<Module> createSwitch(const JsonObject& config);

class Switch : public Module
{

	private:

		volatile float* ptrPV; 			// pointer to the data source
		float 			PV;
		float 			SP;
		bool			mode;			// 0 switch off, 1 switch on
		const char* 	portAndPin;

		Pin 			*pin;


	public:

		Switch(float, volatile float*, const char*, bool);

		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
