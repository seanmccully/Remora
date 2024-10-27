
#include "tmc.h"
#include <cstdint>

#define TOFF_VALUE  4 // [1... 15]

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/
unique_ptr<Module> createTMC2208(const JsonObject& config) {

    const char* comment = config["Comment"];

    const char* RxPin = config["RX pin"];
    float RSense = config["RSense"];
    uint8_t address = config["Address"];
    uint16_t current = config["Current"];
    uint16_t microsteps = config["Microsteps"];
    const char* stealth = config["Stealth chop"];
    uint16_t stall = config["Stall sensitivity"];

    bool stealthchop;

    if (!strcmp(stealth, "on"))
    {
        stealthchop = true;
    }
    else
    {
        stealthchop = false;   
    }

    // SW Serial pin, RSense, mA, microsteps, stealh
    // TMC2208(std::string, float, uint8_t, uint16_t, uint16_t, bool);
    return make_unique<TMC2208>(RxPin, RSense, current, microsteps, stealthchop);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

    // SW Serial pin, RSense, mA, microsteps, stealh, hybrid
    // TMC2209(std::string, float, uint8_t, uint16_t, uint16_t, bool);
TMC2208::TMC2208(const char* rxtxPin, float Rsense, uint16_t mA, uint16_t microsteps, bool stealth) :
    rxtxPin(rxtxPin),
    mA(mA),
    microsteps(microsteps),
    stealth(stealth)
{
    this->Rsense = Rsense;
    this->driver = new TMC2208Stepper(this->rxtxPin, this->rxtxPin, this->Rsense);
}

TMC2208::~TMC2208()
{
    delete this->driver;
}

void TMC2208::configure()
{
    uint16_t result;

    driver->begin();
    
    result = driver->test_connection();
    // Sets the slow decay time (off time) [1... 15]. This setting also limits
    // the maximum chopper frequency. For operation with StealthChop,
    // this parameter is not used, but it is required to enable the motor.
    // In case of operation with StealthChop only, any setting is OK.
    driver->toff(TOFF_VALUE);

    // Comparator blank time. This time needs to safely cover the switching
    // event and the duration of the ringing on the sense resistor. For most
    // applications, a setting of 16 or 24 is good. For highly capacitive
    // loads, a setting of 32 or 40 will be required.
    driver->blank_time(24);

    driver->rms_current(this->mA);
    driver->microsteps(this->microsteps);

    // Toggle spreadCycle on TMC2208/2209/2224: default false, true: much faster!!!!
    driver->en_spreadCycle(!this->stealth);            

    // Needed for StealthChop
    driver->pwm_autoscale(true);             

     driver->iholddelay(10);

    driver->TPOWERDOWN(128);    // ~2s until driver lowers to hold current
    
}

void TMC2208::update()
{
    this->driver->SWSerial->tickerHandler();
}

