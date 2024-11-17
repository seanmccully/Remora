// remora_protocol.h
#ifndef REMORA_PROTOCOL_H
#define REMORA_PROTOCOL_H

#include <cstdint>

// Define bit shifts and masks for the new 11-bit CAN ID format
constexpr uint32_t PRIORITY_SHIFT = 10;  // Priority (1 bit)
constexpr uint32_t SOURCE_SHIFT = 9;    // Source Node ID (1 bit)
constexpr uint32_t DEST_SHIFT = 8;      // Destination Node ID (1 bit)
constexpr uint32_t TYPE_SHIFT = 4;      // Message Type (4 bits)
constexpr uint32_t INDEX_SHIFT = 0;     // Index (4 bits)

constexpr uint32_t PRIORITY_MASK = 0x01;  // 1 bit
constexpr uint32_t SOURCE_MASK = 0x01;   // 1 bit
constexpr uint32_t DEST_MASK = 0x01;     // 1 bit
constexpr uint32_t TYPE_MASK = 0x0F;     // 4 bits
constexpr uint32_t INDEX_MASK = 0x0F;    // 4 bits

/* 29Bit Masks
constexpr uint32_t PRIORITY_SHIFT = 26;
constexpr uint32_t SOURCE_SHIFT = 21;
constexpr uint32_t DEST_SHIFT = 16;
constexpr uint32_t TYPE_SHIFT = 8;
constexpr uint32_t INDEX_SHIFT = 0;

constexpr uint32_t PRIORITY_MASK = 0x07;  // 3 bits
constexpr uint32_t SOURCE_MASK = 0x1F;    // 5 bits
constexpr uint32_t DEST_MASK = 0x1F;      // 5 bits
constexpr uint32_t TYPE_MASK = 0xFF;      // 8 bits
constexpr uint32_t INDEX_MASK = 0xFF;     // 8 bits
*/
// Constants
constexpr size_t MAX_CAN_PAYLOAD = 8;
constexpr int CAN_BAUDRATE = 500000;
constexpr uint32_t CAN_TIMEOUT_MS = 100;
constexpr int MAX_RETRY_COUNT = 3;
constexpr int DIGITAL_INPUTS = 16;
constexpr int DIGITAL_OUTPUTS = 16;


// Define message queue sizes
constexpr size_t HIGH_PRIORITY_QUEUE_SIZE = 16;
constexpr size_t MEDIUM_PRIORITY_QUEUE_SIZE = 32;
constexpr size_t LOW_PRIORITY_QUEUE_SIZE = 16;

typedef struct CANQueueMessage {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
    // Ensure this is defined
    CANQueueMessage& operator=(const CANQueueMessage& other) {
        if (this != &other) {
            id = other.id;
            len = other.len;
            memcpy(data, other.data, sizeof(data));
        }
        return *this;
    }
} CANQueueMessage_t ;

// Message Types
enum MessageType {
    MSG_JOINT_CMD = 0x01,   // Joint command message
    MSG_JOINT_FB = 0x02,    // Joint feedback message
    MSG_SETPOINT = 0x03,    // Setpoint message
    MSG_PV_FB = 0x04,       // Process variable feedback
    MSG_DIG_OUT = 0x05,     // Digital outputs
    MSG_DIG_IN = 0x06,      // Digital inputs
    MSG_STATUS = 0x07,       // Status message
    MSG_JOINT_ENABLE = 0x08,  // Add joint enable message type
    MSG_HEART_BEAT = 0x09,
    MSG_START = 0x0A,
    MSG_START_ACK = 0x0B,
    MSG_STOP = 0x0C,
    MSG_STOP_ACK = 0x0D
};

// Priority Levels
enum Priority {
    PRIO_HIGH = 0,
    PRIO_MEDIUM = 1,
    PRIO_LOW = 2
};

// Node IDs
enum NodeId {
    NODE_CONTROLLER = 0x00,
    NODE_REMORA = 0x01
};

// CAN Frame Structure
#pragma pack(push, 1)

struct CANHeader {
    uint32_t priority : 3;  // Priority level
    uint32_t source : 5;    // Source node ID
    uint32_t dest : 5;      // Destination node ID
    uint32_t type : 8;      // Message type
    uint32_t index : 8;     // Message index/sequence
};

// Define message structures for each type
struct JointCommand {
    float frequency;        // Commanded frequency
};

struct JointFeedback {
    int32_t counts;        // Position feedback in counts
};

struct SetpointMessage {
    float value;           // Setpoint value
};

struct ProcessVariableFb {
    float value;           // Process variable feedback
};

struct DigitalMessage {
    uint8_t values;        // 8 digital values packed into one byte
};

struct StatusMessage {
    uint8_t joint_enables; // Joint enable bits
    uint8_t error_flags;   // Error status flags
};

#pragma pack(pop)

// Function to create a standard 11-bit CAN ID
inline uint16_t makeCanId(uint8_t priority, uint8_t src, uint8_t dest, uint8_t type, uint8_t index) {
    uint16_t id = 0;
    id |= (priority & PRIORITY_MASK) << PRIORITY_SHIFT; // Priority (bit 10)
    id |= (src & SOURCE_MASK) << SOURCE_SHIFT;         // Source Node ID (bit 9)
    id |= (dest & DEST_MASK) << DEST_SHIFT;            // Destination Node ID (bit 8)
    id |= (type & TYPE_MASK) << TYPE_SHIFT;            // Message Type (bits 4–7)
    id |= (index & INDEX_MASK) << INDEX_SHIFT;         // Index (bits 0–3)
    return id;  // Return 11-bit ID
}

// Function to parse a standard 11-bit CAN ID
inline void parseCanId(uint16_t can_id, CANHeader& header) {
    header.priority = (can_id >> PRIORITY_SHIFT) & PRIORITY_MASK;
    header.source = (can_id >> SOURCE_SHIFT) & SOURCE_MASK;
    header.dest = (can_id >> DEST_SHIFT) & DEST_MASK;
    header.type = (can_id >> TYPE_SHIFT) & TYPE_MASK;
    header.index = (can_id >> INDEX_SHIFT) & INDEX_MASK;
}

#endif // REMORA_PROTOCOL_H
