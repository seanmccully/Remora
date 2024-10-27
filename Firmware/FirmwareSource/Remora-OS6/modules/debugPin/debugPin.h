#ifndef DEBUG_H
#define DEBUG_H

#include <cstdint>

#include "module.h"
#include "pin.h"


class Debug : public Module
{

	private:

		bool 		bState;

		Pin*        debugPin;	// class object members - Pin objects

	public:

		Debug(const char*, bool);

		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
