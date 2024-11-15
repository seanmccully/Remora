#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include <vector>
#include <memory>
#include <string>
using namespace std;
#include <atomic>
#include "mbed.h"
#include "module.h"

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
    Timer threadTimer;
    uint64_t nextThreadTime;

    // Statistics
    uint32_t worstCaseThreadTime;
    uint32_t bestCaseThreadTime;

    bool executeModules();
    void waitForNextCycle(uint64_t startTime);
    void updateThreadStats(uint64_t executionTime);
public:

    pruThread(const string& name, uint32_t freq);
    bool registerModule(unique_ptr<Module> module);
    bool startThread();
    void stopThread();
    bool update();
    void pauseThread();
    void resumeThread();
    bool isRunning() const;
    bool isPaused() const;
    uint32_t getWorstCaseTime() const;
    uint32_t getBestCaseTime() const;
    void resetThreadStats();
    const string&getName() const;
    uint32_t getFrequency() const;
    size_t getModuleCount() const;
};


#endif
