#include "pwm.h"
#include "hardwarePwm.h"

#define PID_PWM_MAX 256		// 8 bit resolution

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/

unique_ptr<Module> createPWM(const JsonObject& config) {
    std::unique_ptr<Module> pwm;
    const char* comment = config["Comment"];

    int sp = config["SP[i]"];
    int pwmMax = config["PWM Max"];
    const char* pin = config["PWM Pin"];

    const char* hardware = config["Hardware PWM"];
    const char* variable = config["Variable Freq"];
    int period_sp = config["Period SP[i]"];
    int period = config["Period us"];

    if (!strcmp(hardware,"True"))
    {
        // Hardware PWM
        if (!strcmp(variable,"True"))
        {
            // Variable frequency hardware PWM
             return std::make_unique<HardwarePWM>(rxData.setPoint[period_sp], rxData.setPoint[sp] , period, pin);
        }
        else
        {
            // Fixed frequency hardware PWM
            return std::make_unique<HardwarePWM>(rxData.setPoint[sp], period, pin);
        }
    }
    else
    {
        // Software PWM
        if (pwmMax != 0) // use configuration file value for pwmMax - useful for 12V on 24V systems
        {
            return std::make_unique<PWM>(rxData.setPoint[sp], pin, pwmMax);
        }
        else // use default value of pwmMax
        {
            return std::make_unique<PWM>(rxData.setPoint[sp], pin);
        }
    }
    return pwm;
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

PWM::PWM(volatile float &ptrSP, const char* portAndPin) :
	ptrSP(&ptrSP),
	portAndPin(portAndPin)
{

	this->pwm = new SoftPWM(this->portAndPin);
	this->pwmMax = PID_PWM_MAX-1;
	this->pwm->setMaxPwm(this->pwmMax);
}

// use the following constructor when using 12v devices on a 24v system
PWM::PWM(volatile float &ptrSP, const char* portAndPin, int pwmMax) :
	ptrSP(&ptrSP),
	portAndPin(portAndPin),
	pwmMax(pwmMax)
{
	this->pwm = new SoftPWM(this->portAndPin);
	this->pwm->setMaxPwm(this->pwmMax);
}



void PWM::update()
{
	float SP;

	// update the speed SP
	this->SP = *(this->ptrSP);

    // ensure SP is within range. LinuxCNC PID can have -ve command value
	if (this->SP > 100) this->SP = 100;
    if (this->SP < 0) this->SP = 0;

	// the SP is as a percentage (%)
	// scale the pwm output range (0 - pwmMax) = (0 - 100%)

	SP = this->pwmMax * (this->SP / 100.0);

	this->pwm->setPwmSP(int(SP));

	this->pwm->update();
}

void PWM::slowUpdate()
{
	return;
}
