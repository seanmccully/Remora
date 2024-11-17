// RemoraCAN.cpp
#include "RemoraCAN.h"


RemoraCAN::RemoraCAN(volatile rxData_t* rxData, volatile txData_t* txData, PinName can_rd, PinName can_td)
{
    can = new CAN(can_rd, can_td, CAN_BAUDRATE);
    buffer = new RemoraBuffer(rxData, txData);
}

void RemoraCAN::sendStartAck() {
	CANMessage msg;
	msg.id = makeCanId(PRIO_HIGH, NODE_REMORA, NODE_CONTROLLER, MSG_START_ACK, 0);
	msg.len = 0;  // No payload needed
    buffer->sendControl(msg);
}

void RemoraCAN::sendHeartBeat() {
	CANMessage msg;
	msg.id = makeCanId(PRIO_HIGH, NODE_REMORA, NODE_CONTROLLER, MSG_HEART_BEAT, 0);
	msg.len = 0;  // No payload needed
    canSend(msg);
}

void RemoraCAN::sendStopAck() {
	CANMessage msg;
	msg.id = makeCanId(PRIO_HIGH, NODE_REMORA, NODE_CONTROLLER, MSG_STOP_ACK, 0);
	msg.len = 0;  // No payload needed
    buffer->sendControl(msg);
}

bool RemoraCAN::validateBuffers() const {
	return buffer->isValid(); // Delegate to buffer class
}

bool RemoraCAN::initializeCAN() {
    if (validateBuffers()) {
        can->attach(callback(this, &RemoraCAN::onCanReceived));
    }
    return true;
}

void RemoraCAN::processMessage(CANMessage *msg) {
    CANHeader header;
    parseCanId(msg->id, header);

    switch(header.priority) {
        case PRIO_HIGH:
            handleControlMessage(header, *msg);
            break;
        default:
            buffer->handleReceivedMessage(header, *msg);
            break;
    }
}

void RemoraCAN::handleControlMessage(CANHeader& header, const CANMessage& msg) {

	switch(header.type) {
		case MSG_START:
		    handleStart();
            break;
		case MSG_STOP:
			handleStop();
            break;
		default:
			// Handle immediately
			buffer->handleReceivedMessage(header, msg);
			break;
	}
}

void RemoraCAN::handleStop() {
	stop();
	sendStopAck();
	return;
}

void RemoraCAN::handleStart() {
	start();
	sendStartAck();
	return;
}

void RemoraCAN::reset() {
	can->reset();
}

bool RemoraCAN::sendPacket() {
    // Prepare all messages
    buffer->sendAllData();
    return buffer->sendQueuedMessages(can);
}


void RemoraCAN::onCanReceived() {
    CANMessage msg;
    if (can->read(msg)) {
        CANHeader header;
        parseCanId(msg.id, header);
        switch(header.priority) {
        	case PRIO_HIGH:
           		handleControlMessage(header, msg);
           		break;
            default:
            	buffer->handleReceivedMessage(header, msg);
                break;
        }
	}
}
