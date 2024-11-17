
#ifndef REMORA_H
#define REMORA_H

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include "rtapi_math.h"

#include <fcntl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "RemoraProtocol.h"
// Configuration constants - these should match your system configuration
#define JOINTS          3  // Number of joints
#define VARIABLES       6  // Number of variables
#define DIGITAL_OUTPUTS 16 // Number of digital outputs
#define DIGITAL_INPUTS  16 // Number of digital inputs
#define STEP_MASK      4095
#define STEP_OFFSET    2048
#define STEPBIT        21

// CAN Configuration
#define CAN_BUFF_SIZE  128
#define DEFAULT_CAN_INTERFACE "can0"


#define MODNAME "remora"
#define PREFIX "remora"

#endif
