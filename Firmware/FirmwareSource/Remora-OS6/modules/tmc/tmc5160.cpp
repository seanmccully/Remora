#include "tmc.h"
#include <cstdint>
#include "remora.h"

#define TOFF_VALUE  4 // [1... 15]

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/
unique_ptr<Module> createTMC5160(const JsonObject& config) {

    const char* comment = config["Comment"];
    const char* CSPin = config["CS Pin"];
    const char* SPIBus = config["SPI bus"];
    float       RSense = config["RSense"];
    uint16_t    current = config["Current"];
    uint16_t    microsteps = config["Microsteps"];
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
    // TMC5160(const char*, float, uint8_t, uint16_t, uint16_t, bool);
    return make_unique<TMC5160>(CSPin, SPIBus, RSense, current, microsteps, stealthchop);
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

TMC5160::TMC5160(const char* csPin, const char* spiBus, float Rsense, uint16_t mA, uint16_t microsteps, bool stealth) :
    csPin(csPin),
    spiBus(spiBus),
    mA(mA),
    microsteps(microsteps),
    stealth(stealth)
{
    this->Rsense = Rsense;
    this->driver = new TMC5160Stepper(this->csPin, this->spiBus, this->Rsense, this->mA, this->microsteps, this->stealth);
}

TMC5160::~TMC5160()
{
    delete this->driver;
}


void TMC5160::configure()
{
    uint16_t result;

    result = driver->test_connection();
    // Defaults Set    
}

void TMC5160::update()
{
    this->driver->getMicrostepcounter(); // not sure about this
}

