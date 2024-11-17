// optimized_remora_buffer.h
#ifndef REMORA_BUFFER_H
#define REMORA_BUFFER_H

#include <atomic>
#include <cstdint>
#include <cstring>

#include "mbed.h"

#include "configuration.h"
#include "data.h"
#include "gpioAPI.h"
#include "RemoraProtocol.h"


struct BufferedCANMessage {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    bool used;

    // Constructor from CANMessage
    BufferedCANMessage(const CANMessage& msg) {
        id = msg.id;
        dlc = msg.len;
        memcpy(data, msg.data, dlc);
        used = true;
    }

    // Default constructor
    BufferedCANMessage() : id(0), dlc(0), used(false) {
        memset(data, 0, sizeof(data));
    }

    // Convert to CANMessage
    CANMessage toCANMessage() const {
        CANMessage msg;
        msg.id = id;
        msg.len = dlc;
        memcpy(msg.data, data, dlc);
        return msg;
    }
};


// Ring buffer for CAN frames
template<size_t Size>
class CANBuffer {
private:

    BufferedCANMessage buffer[Size];
    volatile size_t readIndex = 0;
    volatile size_t writeIndex = 0;
    volatile size_t count = 0;

    static constexpr size_t BUFFER_MASK = Size - 1;
    static_assert((Size & (Size - 1)) == 0, "Buffer size must be power of 2");

public:
	CANBuffer() {
		for (auto& msg : buffer) {
			msg.used = false;
			msg.dlc = 0;
		}
	}

	bool write(const CANMessage& msg) {
		if (count >= Size) return false;

	 	buffer[writeIndex] = BufferedCANMessage(msg);
        writeIndex = (writeIndex + 1) & BUFFER_MASK;
        count++;

		return true;
	}
	bool read(CANMessage& msg) {
		if (count == 0) return false;


		if (!buffer[readIndex].used) return false;

		msg = buffer[readIndex].toCANMessage();
		buffer[readIndex].used = false;
		readIndex = (readIndex + 1) & BUFFER_MASK;
		count--;

		return true;
	}
    bool isEmpty() const { return count == 0; }
    bool isFull() const { return count >= Size; }
    size_t available() const { return Size - count; }
};


class RemoraBuffer {
private:
    volatile rxData_t* rxData;
    volatile txData_t* txData;
    // Separate buffers for different message priorities
    CANBuffer<32> highPriorityBuffer;   // Status and critical messages
    CANBuffer<64> mediumPriorityBuffer; // Joint and setpoint data
    CANBuffer<32> lowPriorityBuffer;    // Digital I/O

    // Helper for efficient CAN frame packing
    struct CANFramePacker {
        uint8_t buffer[8];
        size_t offset = 0;

        bool pack(const void* data, size_t size) {
            if (offset + size > 8) return false;
            memcpy(buffer + offset, data, size);
            offset += size;
            return true;
        }

        void reset() { offset = 0; }
    };

    // Helper to queue a message in the appropriate buffer
    bool queueMessage(const CANMessage& msg, Priority priority);

    bool sendJointFeedback(size_t index);
    bool sendDigitalInputs(size_t index);
    bool sendProcessVariable(size_t index);


public:
    RemoraBuffer(volatile rxData_t* rx, volatile txData_t* tx)
        : rxData(rx), txData(tx) {}

	bool prepareJointFeedback();
	bool prepareDigitalInputs();
	bool prepareProcessVariables();

    bool processJointCommand(const CANMessage&);
	bool processDigitalOutputs(const CANMessage&);
    bool processSetpoint(const CANMessage& msg);
    bool processJointEnable(const CANMessage& msg);


    bool sendControl(CANMessage);
	bool handleReceivedMessage(const CANHeader& header, const CANMessage& msg);
	bool sendQueuedMessages(CAN* can);
    bool sendAllData();

    bool isValid() const {
        return rxData != nullptr && txData != nullptr;
    }
};





#endif // REMORA_BUFFER_H
