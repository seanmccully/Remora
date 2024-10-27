#ifndef RESETPIN_H
#define RESETPIN_H

#include <cstdint>

#include "modules/module.h"
#include "drivers/pin/pin.h"

#include "remora.h"

unique_ptr<Module> createResetPin(const JsonObject& config);

class ResetPin : public Module
{
	private:

		volatile bool *ptrReset; 	// pointer to the data source
		const char* portAndPin;

		Pin *pin;

	public:

		ResetPin(volatile bool&, const char*);
		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
