// RemoraCAN.h
#ifndef REMORA_CAN_H
#define REMORA_CAN_H

#include <atomic>
#include <cstdint>
#include <cstring>

#include "mbed.h"

#include "configuration.h"
#include "data.h"
#include "pin.h"
#include "RemoraBuffer.h"


class RemoraCAN {
private:
    //CAN can;
    uint8_t rxData[8];
    RemoraBuffer *buffer;
    CAN *can;
    Pin canRx;
    Pin canTx;

    std::atomic<bool> startReceived{false};
    std::atomic<bool> stopReceived{false};

    // Private helper methods
    bool validateBuffers() const;

	void sendStartAck();
	void sendStopAck();
	void handleControlMessage(CANHeader& header, const CANMessage& msg);
    void handleStart();
    void handleStop();

public:
    RemoraCAN(volatile rxData_t* rxData, volatile txData_t* txData, PinName can_rd, PinName can_td);
    virtual ~RemoraCAN() = default;

    // Delete copy and move operations
    RemoraCAN(const RemoraCAN&) = delete;
    RemoraCAN& operator=(const RemoraCAN&) = delete;
    RemoraCAN(RemoraCAN&&) = delete;
    RemoraCAN& operator=(RemoraCAN&&) = delete;

    bool initializeCAN();
    bool sendPacket();
    void reset();
    void processMessage(CANMessage *msg);
	void sendHeartBeat();
	void onCanReceived();

	inline bool canSend(CANMessage msg) { return can->write(msg); }
    [[nodiscard]] bool hasStarted() const { return startReceived.load(std::memory_order_acquire);  }
    [[nodiscard]] bool hasStopped() const { return stopReceived.load(std::memory_order_acquire); }

    void start() {
        stopReceived.store(false, std::memory_order_release);
        startReceived.store(true, std::memory_order_release);
    }

    void stop() {
        stopReceived.store(true, std::memory_order_release);
        startReceived.store(false, std::memory_order_release);
    }

};
#endif
