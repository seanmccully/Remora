#ifndef BLINK_H
#define BLINK_H

#include <cstdint>
#include <string>

#include "modules/module.h"
#include "drivers/pin/pin.h"

#include "remora.h"

unique_ptr<Module> createBlink(const JsonObject& config);

class Blink : public Module
{

	private:

		bool 		bState;
		uint32_t 	periodCount;
		uint32_t 	blinkCount;

		Pin *blinkPin;	// class object members - Pin objects

	public:

		Blink(const char*, uint32_t, uint32_t);

		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
