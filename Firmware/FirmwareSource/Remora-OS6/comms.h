#ifndef COMMS_H
#define COMMS_H

#include "RemoraComms.h"
// SD card access and Remora communication protocol
// CANBUS Pins PB8 PB9
RemoraComms comms(ptrRxData, ptrTxData, PB_8, PB_9);
