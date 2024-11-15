
#include "remora.h"


#if defined TARGET_SKRV3
#define CAN_TX PB_9
#define CAN_RX PB_8
#elif defined TARGET_OCTOPUS_446
#define CAN_RX PD_0
#define CAN_TX PD_1
#endif

Remora::Remora() 
    {
        this->comms = new RemoraCAN(ptrRxData, ptrTxData, CAN_RX, CAN_TX);
        this->configHandler = new JsonConfigHandler();
        initializeSystem();
    }

bool Remora::initialize() {

    createThreadObjects();
    loadModules();

    return true;
}

void Remora::initializeSystem() {
    configError = false;
    threadsRunning = false;
    base_freq = PRU_BASEFREQ;
    servo_freq = PRU_SERVOFREQ;
    comms_freq = PRU_SERVOFREQ;
}

void Remora::createThreadObjects() {
        threadMap["Base"] = std::make_unique<pruThread>("Base", base_freq);
        threadMap["Servo"] = std::make_unique<pruThread>("Servo", servo_freq);
        threadMap["On load"] = nullptr; // We dont use a thread for On Load functions
}


void Remora::loadModules() {
    ModuleFactory* factory = ModuleFactory::getInstance();
    JsonArray modules = configHandler->getModules();
    if (modules.isNull()) {
      MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"Cannot Load Configuration Modules!",0x1243);
    }

    for (int i=0;i<modules.size();i++) {
        if (modules[i].containsKey("Thread") && modules[i].containsKey("Type")) {
            const char* threadName = modules[i]["Thread"];
            const char* moduleType = modules[i]["Type"];
            // Add frequency to module config to avoid freq in global namespace;
            // Create module using factory
            std::unique_ptr<Module> _mod = factory->createModule(threadName, moduleType, modules[i]);
            if (_mod != nullptr) 
                threadMap[threadName]->registerModule(std::move(_mod));
        } 
    }
}

bool Remora::startThreads() {
        for (const auto& thPair : threadMap) {
            if (thPair.second != nullptr)
                if (!thPair.second->startThread()) {
                    return false;
                }
        }

        threadsRunning = true;
        return true;
}


void Remora::start() {
    Watchdog& watchdog = Watchdog::get_instance();
    watchdog.start(TIMEOUT_WD);

    while (true) {
        watchdog.kick();
        comms->handlePacket();
        handleState();
    }
}

void Remora::transitionState(State _tr) {
    stateMachine.transitionTo(_tr);
}

void Remora::handleState() {
    State currentState = stateMachine.getCurrentState();

    switch (currentState) {
        case State::ST_SETUP:
            if (initialize()) {
                transitionState(State::ST_START);
            } else {
                transitionState(State::ST_ERROR);
            }
            break;

        case State::ST_START:
            if (!threadsRunning) {
                if (!startThreads()) {
                    transitionState(State::ST_ERROR);
                    break;
                }
            }

            if (!PRUreset) {
                transitionState(State::ST_IDLE);
            }
            break;

        case State::ST_IDLE:
            if (comms->getStatus()) {
                transitionState(State::ST_RUNNING);
            }

            if (PRUreset) {
                transitionState(State::ST_WDRESET);
            }
            break;

        case State::ST_RUNNING:
            if (PRUreset) {
                transitionState(State::ST_WDRESET);
            }
            break;

        case State::ST_STOP:
        case State::ST_ERROR:
            cleanup();
            transitionState(State::ST_RESET);
            break;

        case State::ST_RESET:
            cleanup();
            initializeSystem();
            transitionState(State::ST_SETUP);
            break;

        case State::ST_WDRESET:
            cleanup();
            while(1) {} // Force watchdog reset
            break;
    }
}

void Remora::cleanup() {
    for (const auto& thPair : threadMap) {
        if (thPair.second != nullptr)
            thPair.second->stopThread();
    }

    this->threadsRunning = false;
}
