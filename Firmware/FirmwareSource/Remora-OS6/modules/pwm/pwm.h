#ifndef PWM_H
#define PWM_H

#include <cstdint>
//#include <iostream>

#include "modules/module.h"
#include "drivers/softPwm/softPwm.h"

#include "remora.h"

unique_ptr<Module> createPWM(const JsonObject& config);

class PWM : public Module
{

	private:

		volatile float* ptrSP; 			// pointer to the data source
		int 			SP;
		const char* 	portAndPin;
		int 			pwmMax;

		SoftPWM* 		pwm;			// pointer to PWM object - output


	public:

		PWM(volatile float&, const char*);
		PWM(volatile float&, const char*, int);

		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
