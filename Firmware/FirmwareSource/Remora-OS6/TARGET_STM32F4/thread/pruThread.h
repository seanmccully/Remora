#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include <vector>
#include <memory>
#include <string>
#include <atomic>
#include "mbed.h"
#include "module.h"
#include "log.h"

class pruThread {
private:
    // Thread configuration
    std::string threadName;
    uint32_t frequency;
    uint32_t periodUs;

    // Thread control
    std::atomic<bool> threadRunning{false};
    std::atomic<bool> threadPaused{false};

    // Module management
    std::vector<std::unique_ptr<Module>> modules;

    // Timing management
    Timer threadTimer;
    uint64_t nextThreadTime;

    // Statistics
    uint32_t worstCaseThreadTime;
    uint32_t bestCaseThreadTime;

    // Logger
    Logger& logger;

    bool executeModules();
    void waitForNextCycle(uint64_t startTime);
    void updateThreadStats(uint64_t executionTime);
public:

    pruThread(const std::string& name, uint32_t freq, Logger& log);
    bool registerModule(std::unique_ptr<Module> module);
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
    const std::string&getName() const;
    uint32_t getFrequency() const;
    size_t getModuleCount() const;
};

/* Example usage in main loop:
int main() {
    Logger logger("/fs/log.txt");

    // Create threads
    auto baseThread = std::make_unique<pruThread>("Base", 1000, logger);  // 1kHz
    auto servoThread = std::make_unique<pruThread>("Servo", 100, logger); // 100Hz

    // Initialize threads
    baseThread->startThread();
    servoThread->startThread();

    // Main loop
    while(true) {
        // Run thread updates
        baseThread->update();
        servoThread->update();

        // Handle other tasks if needed
        // ...
    }
}
*/
#endif
