#include "pruThread.h"
#include "modules/module.h"
#include "gpioApi.h"

// Statically declare an IRQ handler and run-time enable it
#define armcm_enable_irq(FUNC, NUM, PRIORITY) do {      \
        NVIC_SetVector(NUM, (uint32_t)FUNC);                      \
        NVIC_SetPriority((NUM), (PRIORITY));            \
        NVIC_EnableIRQ((NUM));                          \
    } while (0)

pruThread::pruThread(const string& name, TIM_TypeDef *timer, IRQn_Type irq, uint32_t freq, uint32_t irqHandler, uint8_t prio)
    : threadName(name)
    , timer(timer)
    , irq(irq)
    , frequency(freq)
    , periodUs(1000000 / freq)
{

    armcm_enable_irq(irqHandler, irq, prio);
    //NVIC_SetVector(irq, irqHandler);
    //NVIC_SetPriority(irq, prio);

}



bool pruThread::executeModules() {
    for (const auto& module : modules) {
        if (module) {
            module->runModule();
        }
    }
    return true;
}

bool pruThread::registerModule(unique_ptr<Module> module) {
    if (!module) {
        return false;
    }
    modules.push_back(move(module));
    return true;
}

// For baremetal, this is just initialization
bool pruThread::startThread() {
    if (isRunning()) {
        return true;
    }

    setThreadRunning(true);
    setThreadPaused(false);

    timerPtr = new pruTimer(timer, irq, frequency, this);
    return true;
}

void pruThread::stopThread() {
    setThreadRunning(false);
    setThreadPaused(false);
}

// This is the main function that should be called periodically
bool pruThread::update() {
    if (!isRunning() || isPaused()) {
        return true;
    }

    // Execute all modules
    if (!executeModules()) {
        return false;
    }

    return true;
}

void pruThread::pauseThread() {
    setThreadPaused(true);
}

void pruThread::resumeThread() {
    setThreadPaused(false);
}

size_t pruThread::getModuleCount() const {
    return modules.size();
}

const string& pruThread::getName() const {
    return threadName;
}

uint32_t pruThread::getFrequency() const {
    return frequency;
}
