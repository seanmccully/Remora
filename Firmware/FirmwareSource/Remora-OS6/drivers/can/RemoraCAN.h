// RemoraCAN.h
#ifndef REMORA_CAN_H
#define REMORA_CAN_H

#include "mbed.h"
#include "CAN.h"

#include <atomic>
#include <cstring>
using namespace std;

#include "configuration.h"
#include "data.h"
#include "RemoraProtocol.h"


class RemoraCAN {
private:
    CAN can;
    volatile rxData_t* rxData;
    volatile txData_t* txData;
    Mutex txMutex;
    Mutex rxMutex;
    std::atomic<bool> initialized{false};
    std::atomic<bool> commsActive{false};
    std::atomic<bool> commsError{false};
    
    // Private helper methods
    bool prepareJointCommand(size_t jointIndex, JointCommand& cmd);
    bool validateBuffers() const;
    bool initializeCAN();
    void processJointFeedback(const CANMessage& msg);
    void processProcessVariable(const CANMessage& msg);
    void processDigitalInputs(const CANMessage& msg);
    void processStatusMessage(const CANMessage& msg);
    bool sendJointCommands();
    bool sendSetpoints();
    bool sendDigitalOutputs();
    bool sendEnableStatus();
    
    // CAN message helpers
    bool sendCanMessage(uint32_t id, const void* data, size_t len, int retries = MAX_RETRY_COUNT);
    void handleReceivedMessage(const CANMessage& msg);

public:
    RemoraCAN(volatile rxData_t* rxData, volatile txData_t* txData, PinName can_rd, PinName can_td);
    virtual ~RemoraCAN() = default;
    
    // Delete copy and move operations
    RemoraCAN(const RemoraCAN&) = delete;
    RemoraCAN& operator=(const RemoraCAN&) = delete;
    RemoraCAN(RemoraCAN&&) = delete;
    RemoraCAN& operator=(RemoraCAN&&) = delete;

    void onCanReceived();
    void sendPacket();
    void handlePacket();
    void start();
    void init();
    
    [[nodiscard]] bool isInitialized() const { return initialized.load(std::memory_order_acquire); }
    [[nodiscard]] bool getStatus() const { return commsActive.load(std::memory_order_acquire); }
    [[nodiscard]] bool getError() const { return commsError.load(std::memory_order_acquire); }
    
    void setError(bool error) { commsError.store(error, std::memory_order_release); }
    void resetError() { commsError.store(false, std::memory_order_release); }
    void setStatus(bool status) { commsActive.store(status, std::memory_order_release); }
};
#endif
