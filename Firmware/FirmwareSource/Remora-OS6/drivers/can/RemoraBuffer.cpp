
#include "RemoraBuffer.h"
#include "RemoraCAN.h"


// Similar correction for joint feedback
bool RemoraBuffer::prepareJointFeedback() {

	for (size_t i = 0; i < JOINTS; i++) {
		if (!sendJointFeedback(i))
			return false;
	}
	return true;
}

bool RemoraBuffer::processJointEnable(const CANMessage& msg) {
    if (msg.len != sizeof(uint8_t)) {
                return false;
    }

    // Update the rxData joint enable flags
    rxData->jointEnable = msg.data[0];
    return true;
}

bool RemoraBuffer::processJointCommand(const CANMessage& msg) {
	CANHeader header;
	parseCanId(msg.id, header);

	// mbed can_api.c is improprely bit shifting the msg.length variable
	if (header.index >= JOINTS) {
		return false;
	}

	// Make a non-volatile copy first
	int32_t command;
	memcpy(&command, msg.data, sizeof(int32_t));

	// Then copy to volatile destination
	rxData->jointFreqCmd[header.index] = command;

	return true;
}

// Send all process variables from txData
bool RemoraBuffer::prepareProcessVariables() {

	for (size_t i = 0; i < VARIABLES; i++) {
		if (!sendProcessVariable(i))
			return false;
	}
	return true;
}

// Send digital inputs (from txData)
bool RemoraBuffer::prepareDigitalInputs() {
	CANFramePacker packer;

	for (size_t i = 0; i < (sizeof(uint16_t) + 7) / 8; i++) {
		if (!sendDigitalInputs(i))
			return false;
	}
	return true;
}

// Process received setpoint and store in rxData
bool RemoraBuffer::processSetpoint(const CANMessage& msg) {

	CANHeader header;
	parseCanId(msg.id, header);

	if (header.index >= VARIABLES) {
		return false;
	}

	// Extract the setpoint value
	float setpoint;
	if (msg.len != sizeof(float)) {
		return false;
	}

	memcpy(&setpoint, msg.data, sizeof(float));

	// Store in rxData
	rxData->setPoint[header.index] = setpoint;
	return true;
}


// Process received digital outputs (write to rxData)
bool RemoraBuffer::processDigitalOutputs(const CANMessage& msg) {

	CANHeader header;
	parseCanId(msg.id, header);

	size_t byteIndex = header.index;
	if (byteIndex >= sizeof(uint16_t)) return false;

	uint16_t mask = 0xFF << (byteIndex * 8);
	rxData->outputs &= ~mask;
	rxData->outputs |= (msg.data[0] << (byteIndex * 8)) & mask;

	return true;
}

bool RemoraBuffer::handleReceivedMessage(const CANHeader &header, const CANMessage& msg) {

	switch (header.type) {
		case MSG_JOINT_CMD:
			return processJointCommand(msg);
		case MSG_SETPOINT:
			return processSetpoint(msg);
		case MSG_DIG_OUT:
			return processDigitalOutputs(msg);
        case MSG_JOINT_ENABLE:
            return processJointEnable(msg);
		default:
			return false;
	}
}


bool RemoraBuffer::sendProcessVariable(size_t index) {
	if (index >= VARIABLES) return false;

	CANFramePacker packer;
	packer.reset();

	// Pack the process variable
	const float value = txData->processVariable[index];
	packer.pack(&value, sizeof(float));

	CANMessage msg;
	msg.id = makeCanId(PRIO_MEDIUM, NODE_REMORA, NODE_CONTROLLER, MSG_PV_FB, index);
	msg.len = packer.offset;
	memcpy(msg.data, packer.buffer, packer.offset);

	return queueMessage(msg, PRIO_MEDIUM);
}

bool RemoraBuffer::sendJointFeedback(size_t index) {
	if (index >= JOINTS) return false;
	CANFramePacker packer;
	packer.reset();

	// Pack single joint feedback
	int32_t feedback = txData->jointFeedback[index];
	packer.pack(&feedback, sizeof(int32_t));

	CANMessage msg;
	msg.id = makeCanId(PRIO_MEDIUM, NODE_REMORA, NODE_CONTROLLER, MSG_JOINT_FB, index);
	msg.len = packer.offset;
	memcpy(msg.data, packer.buffer, packer.offset);

	return queueMessage(msg, PRIO_MEDIUM);
}

bool RemoraBuffer::sendDigitalInputs(size_t index) {

	if (index >= DIGITAL_INPUTS) return false;
	CANFramePacker packer;
	packer.reset();

	uint8_t inputByte = (txData->inputs >> (index * 8)) & 0xFF;
	packer.pack(&inputByte, sizeof(uint8_t));

	CANMessage msg;
	msg.id = makeCanId(PRIO_LOW, NODE_REMORA, NODE_CONTROLLER, MSG_DIG_IN, index);
	msg.len = packer.offset;
	memcpy(msg.data, packer.buffer, packer.offset);

	return queueMessage(msg, PRIO_LOW);
}

bool RemoraBuffer::sendControl(CANMessage msg) {
    return queueMessage(msg, PRIO_HIGH);
}

// Call this to send all data
bool RemoraBuffer::sendAllData() {
	// Send process variables
	prepareProcessVariables();
	// Send joint feedback
	prepareJointFeedback();
	// Send digital inputs
	prepareDigitalInputs();

    return true;
}

// Queue a message to the appropriate priority buffer
bool RemoraBuffer::queueMessage(const CANMessage& msg, Priority priority) {
	switch (priority) {
		case PRIO_HIGH:
			if (!highPriorityBuffer.write(msg)) {
				return false;
			}
			break;

		case PRIO_MEDIUM:
			if (!mediumPriorityBuffer.write(msg)) {
				return false;
			}
			break;

		case PRIO_LOW:
			if (!lowPriorityBuffer.write(msg)) {
				return false;
			}
			break;

		default:
			return false;
	}
	return true;
}

// Send messages from all buffers in priority order
bool RemoraBuffer::sendQueuedMessages(CAN *can) {
	CANMessage msg;

    unsigned char err = NULL;
	// Send high priority messages first
	while (highPriorityBuffer.read(msg)) {
		if (!can->write(msg)) {
            can->reset();
		}
        HAL_Delay(2);
	}

	// Then medium priority
	while (mediumPriorityBuffer.read(msg)) {
		if (!can->write(msg)) {
            can->reset();
		}
        HAL_Delay(2);
	}

	// Finally low priority
	while (lowPriorityBuffer.read(msg)) {
		if (!can->write(msg)) {
            can->reset();
		}
        HAL_Delay(2); // sending too many messages at once
	}
	return err == NULL ? true : false;
}
