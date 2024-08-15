#include "tmc.h"
#include <cstdint>
#include <string>

#define TOFF_VALUE  4 // [1... 15]

/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON     
************************************************************************/
void createTMC5160()
{
    printf("Make TMC5160\n");

    const char* comment = module["Comment"];
    printf("%s\n",comment);

    const char* CSPin = module["CS Pin"];
    const char* SPIBus = module["SPI bus"];
    float       RSense = module["RSense"];
    uint16_t    current = module["Current"];
    uint16_t    microsteps = module["Microsteps"];
    const char* stealth = module["Stealth chop"];
    uint16_t stall = module["Stall sensitivity"];

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
    // TMC5160(std::string, float, uint8_t, uint16_t, uint16_t, bool);
    TMC5160* tmc = new TMC5160(string(CSPin), string(SPIBus), RSense, current, microsteps, stealthchop);

    printf("\nStarting the COMMS thread\n");
    commsThread->startThread();
    commsThread->registerModule(tmc);

    tmc->configure();

    printf("\nStopping the COMMS thread\n");
    commsThread->stopThread();
    commsThread->unregisterModule(tmc);

    delete tmc;
}


/***********************************************************************
                METHOD DEFINITIONS
************************************************************************/

TMC5160::TMC5160(std::string csPin, std::string spiBus, float Rsense, uint16_t mA, uint16_t microsteps, bool stealth) :
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

    printf("Testing connection to TMC driver...");
    result = driver->test_connection();
    if (result) {
        printf("failed!\n");
        printf("Likely cause: ");
        switch(result) {
            case 1: printf("loose connection\n"); break;
            case 2: printf("no power\n"); break;
        }
        printf("  Fix the problem and reset board.\n");
        //abort();
    }
    else   
    {
        printf("OK\n");
    }

    // Defaults Set    
}

void TMC5160::update()
{
    this->driver->getMicrostepcounter(); // not sure about this
}

