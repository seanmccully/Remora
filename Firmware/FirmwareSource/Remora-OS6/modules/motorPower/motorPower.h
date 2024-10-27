#ifndef MOTORPOWER_H
#define MOTORPOWER_H

#include <cstdint>

#include "modules/module.h"
#include "drivers/pin/pin.h"

#include "remora.h"

unique_ptr<Module> createMotorPower(const JsonObject& config);

class MotorPower : public Module
{
	private:

		const char* portAndPin;

		Pin *pin;

	public:

        MotorPower(const char*);
		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
