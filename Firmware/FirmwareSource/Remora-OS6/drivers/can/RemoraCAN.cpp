// RemoraCAN.cpp
#include "RemoraCAN.h"
#include <chrono>


RemoraCAN::RemoraCAN(volatile rxData_t* rxData, volatile txData_t* txData, PinName can_rd, PinName can_td) :
    can(can_rd, can_td, CAN_BAUDRATE),
    rxData(rxData),
    txData(txData)
{

    if (!validateBuffers()) {
        printf("Invalid buffer pointers\n");
        setError(true);
        return;
    }

    if (!initializeCAN()) {
        setError(true);
        return;
    }

    can.attach(callback(this, &RemoraCAN::onCanReceived));
    initialized.store(true, memory_order_release);
}

bool RemoraCAN::validateBuffers() const {
    return rxData != nullptr && txData != nullptr;
}

bool RemoraCAN::initializeCAN() {
    return can.frequency(CAN_BAUDRATE);
}

bool RemoraCAN::sendCanMessage(uint32_t id, const void* data, size_t len, int retries) {
    if (len > MAX_CAN_PAYLOAD) {
        printf("Message too long for CAN frame\n");
        return false;
    }

    CANMessage msg;
    msg.id = id;
    msg.len = static_cast<unsigned char>(len);
    memcpy(msg.data, data, len);

    for (int retry = 0; retry < retries; retry++) {
        if (can.write(msg)) {
            return true;
        }
        ThisThread::sleep_for(10ms);
    }
    
    printf("CAN transmission failed after retries\n");
    return false;
}

void RemoraCAN::processJointFeedback(const CANMessage& msg) {
    CANHeader header;
    parseCanId(msg.id, header);
    
    if (header.index >= JOINTS || msg.len != sizeof(JointFeedback)) {
        setError(true);
        return;
    }

    JointFeedback feedback;
    memcpy(&feedback, msg.data, sizeof(JointFeedback));
    txData->jointFeedback[header.index] = feedback.counts;
}

void RemoraCAN::processProcessVariable(const CANMessage& msg) {
    CANHeader header;
    parseCanId(msg.id, header);
    
    if (header.index >= VARIABLES || msg.len != sizeof(ProcessVariableFb)) {
        setError(true);
        return;
    }

    ProcessVariableFb pv;
    memcpy(&pv, msg.data, sizeof(ProcessVariableFb));
    txData->processVariable[header.index] = pv.value;
}

void RemoraCAN::processDigitalInputs(const CANMessage& msg) {
    CANHeader header;
    parseCanId(msg.id, header);
    
    size_t baseInput = header.index * 8;
    if (baseInput >= DIGITAL_INPUTS) {
        setError(true);
        return;
    }

    DigitalMessage digital;
    memcpy(&digital, msg.data, sizeof(DigitalMessage));
    
    // Update input bits
    uint16_t inputMask = 0xFF << baseInput;
    txData->inputs &= ~inputMask;
    txData->inputs |= (digital.values << baseInput) & inputMask;
}

void RemoraCAN::processStatusMessage(const CANMessage& msg) {
    StatusMessage status;
    memcpy(&status, msg.data, sizeof(StatusMessage));
    
    setStatus(status.joint_enables != 0);
    if (status.error_flags) {
        setError(true);
    }
}

void RemoraCAN::handleReceivedMessage(const CANMessage& msg) {
    CANHeader header;
    parseCanId(msg.id, header);
    
    if (header.dest != NODE_REMORA) {
        return;
    }

    switch (header.type) {
        case MSG_JOINT_FB:
            processJointFeedback(msg);
            break;
            
        case MSG_PV_FB:
            processProcessVariable(msg);
            break;
            
        case MSG_DIG_IN:
            processDigitalInputs(msg);
            break;
            
        case MSG_STATUS:
            processStatusMessage(msg);
            break;
            
        default:
            printf("Unknown message type: %d\n", header.type);
            break;
    }
}

void RemoraCAN::onCanReceived() {
    if (!isInitialized()) {
        printf("Received data before initialization\n");
        return;
    }

    ScopedLock<Mutex> lock(rxMutex);
    
    CANMessage msg;
    if (can.read(msg)) {
        handleReceivedMessage(msg);
    }
}
bool RemoraCAN::prepareJointCommand(size_t jointIndex, JointCommand& cmd) {
    if (jointIndex >= JOINTS) {
        printf("Invalid joint index: %zu\n", jointIndex);
        return false;
    }
    
    // Explicit conversion and assignment
    cmd.frequency = rxData->jointFreqCmd[jointIndex];
    
    // Optional: Add validation
    if (!isfinite(cmd.frequency)) {
        printf("Invalid frequency value for joint %zu\n", jointIndex);
        return false;
    }
    
    return true;
}

// In RemoraCAN.cpp - Update sendJointCommands()
bool RemoraCAN::sendJointCommands() {
    for (size_t i = 0; i < JOINTS; i++) {
        JointCommand cmd;

        // Prepare the command with validation
        if (!prepareJointCommand(i, cmd)) {
            setError(true);
            return false;
        }

        // Create CAN message ID
        uint32_t id = makeCanId(PRIO_MEDIUM, NODE_REMORA, NODE_CONTROLLER,
                               MSG_JOINT_CMD, static_cast<uint8_t>(i));

        // Send the command
        if (!sendCanMessage(id, &cmd, sizeof(cmd))) {
            printf("Failed to send joint command for joint %zu\n", i);
            setError(true);
            return false;
        }
    }
    return true;
}

bool RemoraCAN::sendSetpoints() {
    for (size_t i = 0; i < VARIABLES; i++) {
        SetpointMessage sp{rxData->setPoint[i]};
        uint32_t id = makeCanId(PRIO_MEDIUM, NODE_REMORA, NODE_CONTROLLER, MSG_SETPOINT, i);
        
        if (!sendCanMessage(id, &sp, sizeof(sp))) {
            return false;
        }
    }
    return true;
}

bool RemoraCAN::sendDigitalOutputs() {
    for (size_t i = 0; i < (DIGITAL_OUTPUTS + 7) / 8; i++) {
        DigitalMessage digital;
        digital.values = (rxData->outputs >> (i * 8)) & 0xFF;
        
        uint32_t id = makeCanId(PRIO_LOW, NODE_REMORA, NODE_CONTROLLER, MSG_DIG_OUT, i);
        if (!sendCanMessage(id, &digital, sizeof(digital))) {
            return false;
        }
    }
    return true;
}

bool RemoraCAN::sendEnableStatus() {
    StatusMessage status;
    status.joint_enables = rxData->jointEnable;
    status.error_flags = getError() ? 1 : 0;
    
    uint32_t id = makeCanId(PRIO_HIGH, NODE_REMORA, NODE_CONTROLLER, MSG_STATUS, 0);
    return sendCanMessage(id, &status, sizeof(status));
}

void RemoraCAN::sendPacket() {
    if (!isInitialized()) {
        printf("Attempted to send before initialization\n");
        return;
    }

    ScopedLock<Mutex> lock(txMutex);
    
    if (!sendJointCommands() || !sendSetpoints() || !sendDigitalOutputs() || !sendEnableStatus()) {
        setError(true);
    }
}

void RemoraCAN::handlePacket() {
    ScopedLock<Mutex> lock(rxMutex);
    if (getStatus()) {
        // Process any buffered data if needed
        commsActive.store(false, memory_order_release);
    }
}

void RemoraCAN::start() {
    if (!isInitialized()) {
        printf("Cannot start: not initialized\n");
        return;
    }
    txData->header = PRU_DATA;
    sendPacket();
}

void RemoraCAN::init() {
    ScopedLock<Mutex> lock(txMutex);
    setStatus(true);
    resetError();
}
