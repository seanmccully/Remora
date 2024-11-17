
#include "remora.h"
#include "irqHandlers.h"

Remora::Remora()
    {
        this->configHandler = new JsonConfigHandler();

        baseThread = make_unique<pruThread>("Base", TIM3, TIM3_IRQn, base_freq, (uint32_t)TIM3_IRQHandler, 2);
        servoThread = make_unique<pruThread>("Servo", TIM5, TIM5_IRQn, servo_freq, (uint32_t)TIM5_IRQHandler, 3);

        transitionState(State::ST_SETUP);
        handleState();
    }

bool Remora::initialize() {
    initializeSystem(); // Setup system
    loadModules(); // Read the config, and load modules.
    transitionState(State::ST_START); // Transition
    return true;
}

void Remora::initializeSystem() {
    comms = new RemoraCAN(rxData, txData, CAN_RX, CAN_TX);
    comms->initializeCAN(); // attach rx Message handler
    configError = false;
    threadsRunning = false;
    base_freq = PRU_BASEFREQ;
    servo_freq = PRU_SERVOFREQ;
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
            int threadIndex = 0;

            // Add frequency to module config to avoid freq in global namespace;
            // Create module using factory
            std::unique_ptr<Module> _mod = factory->createModule(threadName, moduleType, modules[i]);
            if (strcmp(threadName, "Servo") == 0)
                servoThread->registerModule(move(_mod));
            else if (strcmp(threadName, "Base") == 0)
                baseThread->registerModule(move(_mod));
            else
                onLoad.push_back(move(_mod));

        }
    }

}

bool Remora::startThreads() {

        servoThread->startThread();
        baseThread->startThread();
        for (const auto& module : onLoad) {
            if (module) {
                module->configure();
            }
        }
        threadsRunning = true;
        return true;
}

void Remora::run() {

    Watchdog& watchdog = Watchdog::get_instance();
    watchdog.start(TIMEOUT_WD);
    while (true) {
        watchdog.kick();
        comms->sendPacket();
        if (comms->hasStopped()) {
            transitionState(State::ST_IDLE);
            return;
        }
    }
}

void Remora::start() {

    if (comms->hasStarted()) {
        run();
    } else {
        //comms->sendHeartBeat();
        transitionState(State::ST_IDLE);
        HAL_Delay(10);
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
            transitionState(State::ST_RUNNING);

            break;

        case State::ST_IDLE:
            transitionState(State::ST_RUNNING);

            if (PRUreset) {
                transitionState(State::ST_WDRESET);
            }
            break;

        case State::ST_RUNNING:
            if (PRUreset) {
                transitionState(State::ST_WDRESET);
            }
            start();
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
    servoThread->stopThread();
    baseThread->stopThread();
    this->threadsRunning = false;
}

bool Remora::isError() {

    State currentState = stateMachine.getCurrentState();

    if (currentState == State::ST_ERROR || currentState == State::ST_WDRESET) {
        return true;
    } else {
        return false;
    }
}
