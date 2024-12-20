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
#elif defined TARGET_SKRV2 || TARGET_OCTOPUS_446 || TARGET_BLACK_F407VE || TARGET_OCTOPUS_429 | TARGET_SKRV3
#include "SDMMCBlockDevice.h"
#endif

// pointers to data
static volatile rxData_t*  ptrRxData = &rxData;
static volatile txData_t*  ptrTxData = &txData;

//#include "comms.h" // Include after ptrRxData Defined

static uint8_t resetCnt;
// boolean
static volatile bool PRUreset;


class Remora {
private:
    StateMachine stateMachine;
    JsonConfigHandler* configHandler;
    RemoraCAN* comms;

    // Thread pointers
    static std::unique_ptr<pruThread> baseThread;
    static std::unique_ptr<pruThread> servoThread;
    static std::unique_ptr<pruThread> commsThread;
    map<std::string,std::unique_ptr<pruThread>> threadMap; // = {{ string("Base"), baseThread }, { string("Servo"), servoThread }, {string("On Load"), commsThread}};

    // Configuration state
    bool configError;
    bool threadsRunning;
    uint32_t base_freq;
    uint32_t servo_freq;
    uint32_t comms_freq;

    void initializeSystem();
    bool loadConfiguration();
    void createThreadObjects();
    // Example usage in loadModules function
    void loadModules();
    bool startThreads();
    void transitionState(State _tr);
    void handleState(void);
    bool initialize(void);
public:
    Remora();
    void start(void);
    void cleanup(void);
};
#endif
