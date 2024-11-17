#ifndef REMORA_H
#define REMORA_H

#include <map>
#include <memory>

#include "mbed.h"
#include "FATFileSystem.h"

#include "configuration.h"
#include "states.h"
#include "jsonConfigHandler.h"
#include "moduleFactory.h"
#include "pruThread.h"
#include "pruThread.h"
#include "data.h"

#include "RemoraCAN.h"
const uint32_t TIMEOUT_WD=Watchdog::get_instance().get_max_timeout() - 1;

#if defined TARGET_LPC176X || TARGET_STM32F1 || TARGET_SPIDER || TARGET_SPIDER_KING || TARGET_MONSTER8 || TARGET_ROBIN_3 || TARGET_MANTA8
#include "SDBlockDevice.h"
#elif defined TARGET_SKRV2 || TARGET_OCTOPUS_446 || TARGET_BLACK_F407VE || TARGET_OCTOPUS_429 || TARGET_SKRV3 || TARGET_OCTOPUS_723
#include "SDMMCBlockDevice.h"
#endif

// pointers to data
//#include "comms.h" // Include after ptrRxData Defined

static uint8_t resetCnt;


class Remora {
private:
    StateMachine stateMachine;
    JsonConfigHandler* configHandler;
    RemoraCAN* comms;
    // Thread pointers
    std::unique_ptr<pruThread> baseThread;
    std::unique_ptr<pruThread> servoThread;
    vector<unique_ptr<Module>> onLoad;


    // Configuration state
    bool configError;
    bool threadsRunning;
    uint32_t base_freq;
    uint32_t servo_freq;
    uint32_t comms_freq;

    void initializeSystem();
    bool loadConfiguration();

    void loadModules();
    bool startThreads();
    void transitionState(State _tr);
    bool initialize(void);
    void start(void);
    void run(void);
public:
    Remora();
    void handleState(void);
    bool isError();
    void cleanup(void);
};
#endif
