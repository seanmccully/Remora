
#include "moduleList.h"
#include "moduleFactory.h"


ModuleCreator ModuleFactory::createBaseModule(const char* modN) {

    if (strcmp(modN,"Stepgen") == 0)
        return createStepgen;

    if (strcmp(modN,"Encoder") == 0)
        return createEncoder;

    if (strcmp(modN,"RCServo") == 0)
        return createRCServo;
    return nullptr;
}

// Servo thread modules
ModuleCreator ModuleFactory::createServoModule(const char* modN) {

    if (strcmp(modN,"eStop") == 0)
        return createEStop;

    if (strcmp(modN,"Digital Pin") == 0)
        return createDigitalPin;

    if (strcmp(modN,"PWM") == 0)
        return createPWM;

    if (strcmp(modN,"Temperature") == 0)
        return createTemperature;

    if (strcmp(modN,"Switch") == 0)
        return createSwitch;

    if (strcmp(modN,"QEI") == 0)
        return createQEI;

    if (strcmp(modN,"Blink") == 0)
        return createBlink;

    if (strcmp(modN,"Reset Pin") == 0)
        return createResetPin;

    return nullptr;
}

    // On load modules
ModuleCreator ModuleFactory::createOnLoadModule(const char* modN) {

    if (strcmp(modN, "MCP4451") == 0)
        return createMCP4451;

    if (strcmp(modN,"Motor Power") == 0)
        return createMotorPower;

    if (strcmp(modN,"TMC2208") == 0)
        return createTMC2208;

    if (strcmp(modN,"TMC2209") == 0)
        return createTMC2209;

    if (strcmp(modN,"TMC5160") == 0)
        return createTMC5160;

    return nullptr;
}

// Create module based on thread and type
std::unique_ptr<Module> ModuleFactory::createModule(const char* _tname,
                                   const char* _mtype,
                                   const JsonVariant config) {
    if (strcmp(_tname,"Base") == 0)
        return createBaseModule(_mtype)(config);
    if (strcmp(_tname,"Servo") == 0)
        return createServoModule(_mtype)(config);
    if (strcmp(_tname,"On load") == 0)
        return createOnLoadModule(_mtype)(config); // Dont return On Load modules

    return nullptr;
}

// Static instance accessor
ModuleFactory* ModuleFactory::getInstance() {
    static ModuleFactory* instance = new ModuleFactory();
    return instance;
}
