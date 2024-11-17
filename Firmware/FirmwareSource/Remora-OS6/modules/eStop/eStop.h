#ifndef ESTOP_H
#define ESTOP_H

#include <cstdint>
#include <iostream>

#include "../../configuration.h"
#include "../../remora.h"
#include "modules/module.h"
#include "drivers/pin/pin.h"

#include "remora.h"

unique_ptr<Module> createEStop(const JsonObject& config);

class eStop : public Module
{

	private:

        volatile uint32_t *ptrTxHeader;
		const char* 	portAndPin;

        Pin *pin;


	public:

		eStop(volatile uint32_t*, const char*);

		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
