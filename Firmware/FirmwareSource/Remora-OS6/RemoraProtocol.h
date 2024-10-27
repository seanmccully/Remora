// remora_protocol.h
#ifndef REMORA_PROTOCOL_H
#define REMORA_PROTOCOL_H

#include <cstdint>

// Message Types
enum MessageType {
    MSG_JOINT_CMD = 0x01,   // Joint command message
    MSG_JOINT_FB = 0x02,    // Joint feedback message
    MSG_SETPOINT = 0x03,    // Setpoint message
    MSG_PV_FB = 0x04,       // Process variable feedback
    MSG_DIG_OUT = 0x05,     // Digital outputs
    MSG_DIG_IN = 0x06,      // Digital inputs
    MSG_STATUS = 0x07       // Status message
};

// Priority Levels
enum Priority {
    PRIO_HIGH = 0,
    PRIO_MEDIUM = 1,
    PRIO_LOW = 2
};

// Node IDs
enum NodeId {
    NODE_CONTROLLER = 0x01,
    NODE_REMORA = 0x02
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

// Helper functions
inline uint32_t makeCanId(uint8_t priority, uint8_t src, uint8_t dest, uint8_t type, uint8_t index) {
    return ((uint32_t)priority << 26) |
           ((uint32_t)src << 21) |
           ((uint32_t)dest << 16) |
           ((uint32_t)type << 8) |
           (uint32_t)index;
}

inline void parseCanId(uint32_t can_id, CANHeader& header) {
    header.priority = (can_id >> 26) & 0x07;
    header.source = (can_id >> 21) & 0x1F;
    header.dest = (can_id >> 16) & 0x1F;
    header.type = (can_id >> 8) & 0xFF;
    header.index = can_id & 0xFF;
}

// Constants
constexpr size_t MAX_CAN_PAYLOAD = 8;
constexpr int CAN_BAUDRATE = 500000;  // 500 kbps
constexpr uint32_t CAN_TIMEOUT_MS = 100;
constexpr int MAX_RETRY_COUNT = 3;
constexpr int DIGITAL_INPUTS = 16;
constexpr int DIGITAL_OUTPUTS = 16;

#endif // REMORA_PROTOCOL_H
