#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include <cstdint>

#include "jsonConfigHandler.h"
#include "module.h"
#include "pin.h"

#include "remora.h"
#include "data.h"

unique_ptr<Module> createDigitalPin(const JsonObject& config);

class DigitalPin : public Module
{
	private:

		volatile uint16_t *ptrData; 	// pointer to the data source
		int bitNumber;				// location in the data source
		bool invert;
		int mask;

		int mode;
        int modifier;
		const char* portAndPin;

		Pin *pin;

	public:

    DigitalPin(volatile uint16_t&, int, const char*, int, bool);
		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
