#include "pruThread.h"
#include "modules/module.h"



pruThread::pruThread(const string& name, uint32_t freq)
    : threadName(name)
    , frequency(freq)
    , periodUs(1000000 / freq)
    , worstCaseThreadTime(0)
    , bestCaseThreadTime(0)
{}


bool pruThread::executeModules() {
    for (const auto& module : modules) {
        if (module) { 
            module->runModule();    
        }
    }
    return true;
}

void pruThread::waitForNextCycle(uint64_t startTime) {
    nextThreadTime += periodUs;

    // Check for thread overrun
    if (nextThreadTime < startTime) {
        nextThreadTime = startTime + periodUs;
    }

    // Wait until next cycle
    while (threadTimer.read_us() < nextThreadTime) {
        __WFE(); // Wait for event (power saving)
    }
}

void pruThread::updateThreadStats(uint64_t executionTime) {
    if (executionTime > worstCaseThreadTime) {
        worstCaseThreadTime = executionTime;
    }
    if (executionTime < bestCaseThreadTime || bestCaseThreadTime == 0) {
        bestCaseThreadTime = executionTime;
    }
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
    if (threadRunning) {
        return true;
    }

    threadRunning = true;
    threadPaused = false;
    threadTimer.start();
    nextThreadTime = threadTimer.read_us();

    return true;
}

void pruThread::stopThread() {
    threadRunning = false;
    threadPaused = false;
}

// This is the main function that should be called periodically
bool pruThread::update() {
    if (!threadRunning || threadPaused) {
        return true;
    }

    uint64_t startTime = threadTimer.read_us();

    // Execute all modules
    if (!executeModules()) {
        return false;
    }

    // Calculate execution time
    uint64_t executionTime = threadTimer.read_us() - startTime;
    updateThreadStats(executionTime);

    // Wait for next cycle
    waitForNextCycle(startTime);

    return true;
}

void pruThread::pauseThread() {
    threadPaused = true;
}

void pruThread::resumeThread() {
    threadPaused = false;
}

bool pruThread::isRunning() const {
    return threadRunning;
}

bool pruThread::isPaused() const {
    return threadPaused;
}

uint32_t pruThread::getWorstCaseTime() const {
    return worstCaseThreadTime;
}

uint32_t pruThread::getBestCaseTime() const {
    return bestCaseThreadTime;
}

void pruThread::resetThreadStats() {
    worstCaseThreadTime = 0;
    bestCaseThreadTime = 0;
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
