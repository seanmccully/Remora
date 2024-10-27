#include "pruThread.h"
#include "modules/module.h"


using namespace std;

pruThread::pruThread(const std::string& name, uint32_t freq, Logger& log)
    : threadName(name)
    , frequency(freq)
    , periodUs(1000000 / freq)
    , worstCaseThreadTime(0)
    , bestCaseThreadTime(0)
    , logger(log) {

    logger.log(LOG_INFO, "Created thread %s with frequency %d Hz (period %d us)\n",
              threadName.c_str(), frequency, periodUs);
}


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
        logger.log(LOG_WARN, "Thread %s overrun!\n", threadName.c_str());
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

bool pruThread::registerModule(std::unique_ptr<Module> module) {
    if (!module) {
        logger.log(LOG_ERROR, "Attempted to register null module in thread %s\n",
                  threadName.c_str());
        return false;
    }
    modules.emplace_back(std::move(module));
    logger.log(LOG_INFO, "Registered module in thread %s\n", threadName.c_str());
    return true;
}

// For baremetal, this is just initialization
bool pruThread::startThread() {
    if (threadRunning) {
        logger.log(LOG_WARN, "Thread %s already running\n", threadName.c_str());
        return true;
    }

    threadRunning = true;
    threadPaused = false;
    threadTimer.start();
    nextThreadTime = threadTimer.read_us();

    logger.log(LOG_INFO, "Started thread %s\n", threadName.c_str());
    return true;
}

void pruThread::stopThread() {
    threadRunning = false;
    threadPaused = false;
    logger.log(LOG_INFO, "Stopped thread %s\n", threadName.c_str());
}

// This is the main function that should be called periodically
bool pruThread::update() {
    if (!threadRunning || threadPaused) {
        return true;
    }

    uint64_t startTime = threadTimer.read_us();

    // Execute all modules
    if (!executeModules()) {
        logger.log(LOG_ERROR, "Module execution failed in thread %s\n", threadName.c_str());
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
    logger.log(LOG_INFO, "Paused thread %s\n", threadName.c_str());
}

void pruThread::resumeThread() {
    threadPaused = false;
    logger.log(LOG_INFO, "Resumed thread %s\n", threadName.c_str());
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

const std::string& pruThread::getName() const {
    return threadName;
}

uint32_t pruThread::getFrequency() const {
    return frequency;
}
