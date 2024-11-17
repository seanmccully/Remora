#include "tmc.h"
#include <cstdint>
#include "remora.h"

#define TOFF_VALUE  4 // [1... 15]

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON
************************************************************************/
unique_ptr<Module> createTMC5160(const JsonObject& config) {

    const char* CSPin = config["CS pin"];
    float       RSense = config["RSense"];
    uint16_t    current = config["Current"];
    uint8_t    microsteps = config["Microsteps"];
    const char* stealth = config["Stealth chop"];
    uint8_t stall = config["Stall sensitivity"];
    bool stealthchop;

    if (strcmp(stealth, "off") == 0)
    {
        stealthchop = false;
    }
    else
    {
        stealthchop = true;
    }

    if (config.containsKey("SPI bus")) {
        const char* SPIBus = config["SPI bus"];
        return make_unique<TMC5160>(CSPin, SPIBus, RSense, current, microsteps, stealthchop);
    }
    else if (config.containsKey("MISO") && config.containsKey("MOSI") && config.containsKey("SCK")) {
        const char* miso = config["MISO"];
        const char* mosi = config["MOSI"];
        const char* sck = config["SCK"];
        return make_unique<TMC5160>(CSPin, miso, mosi, sck, RSense, current, microsteps, stealthchop);
    }
    MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"Invalid TMC5160 Configuration",0x1345);
    return nullptr;
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

TMC5160::TMC5160(const char* csPin, const char* miso, const char* mosi, const char* sck, float Rsense, uint16_t mA, uint16_t microsteps, bool stealth) :
    csPin(csPin),
    miso(miso),
    mosi(mosi),
    sck(sck),
    mA(mA),
    microsteps(microsteps),
    stealth(stealth)
{
    this->Rsense = Rsense;
    this->driver = new TMC5160Stepper(this->csPin, this->miso, this->mosi, this->sck, this->Rsense, this->mA, this->microsteps, this->stealth);
}

TMC5160::~TMC5160()
{
    delete this->driver;
}


void TMC5160::configure()
{
    driver->begin();
}

void TMC5160::update()
{
    this->driver->getMicrostepcounter(); // not sure about this
}

