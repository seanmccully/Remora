#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include <vector>
#include <memory>
#include <string>
using namespace std;
#include <atomic>
#include "mbed.h"
#include "module.h"
#include "timer.h"

class pruThread {
private:
    // Thread configuration
    string threadName;
    uint32_t frequency;
    uint32_t periodUs;

    // Thread control
    atomic<bool> threadRunning{false};
    atomic<bool> threadPaused{false};

    // Module management
    vector<unique_ptr<Module>> modules;

    // Timing management
    pruTimer* timerPtr;
    TIM_TypeDef* timer;
    IRQn_Type    irq;
    uint64_t nextThreadTime;

    [[ nodiscard ]] void setThreadRunning(bool val) { threadRunning.store(val, std::memory_order_release); }
    [[ nodiscard ]] void setThreadPaused(bool val) { threadPaused.store(val, std::memory_order_release); }

    bool executeModules();
    void waitForNextCycle(uint64_t startTime);
public:

    pruThread(const string& name, TIM_TypeDef *timer, IRQn_Type irq, uint32_t freq, uint32_t irqHandler, uint8_t prio);
    bool registerModule(unique_ptr<Module> module);
    void startModules();

    [[nodiscard]] bool isRunning() const { return threadRunning.load(std::memory_order_acquire);  }
    [[nodiscard]] bool isPaused() const { return threadPaused.load(std::memory_order_acquire); }



    bool startThread();

    void stopThread();
    bool update();
    void pauseThread();
    void resumeThread();
    const string&getName() const;
    uint32_t getFrequency() const;
    size_t getModuleCount() const;
};


#endif
