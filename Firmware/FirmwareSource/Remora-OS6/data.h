#ifndef DATA_H
#define DATA_H

#include <stdint.h>  // Add this line for uint8_t type
#include "configuration.h"

class RemoraCAN;

#pragma pack(push, 1)
typedef union rxData_t
{
  // this allow structured access to the incoming SPI data without having to move it
  struct
  {
    uint8_t rxBuffer[DATA_BUFF_SIZE+2];
  };
  struct
  {
    uint32_t header;
    volatile uint32_t jointFreqCmd[JOINTS]; 	// Base thread commands ?? - basically motion
    float setPoint[VARIABLES];		  // Servo thread commands ?? - temperature SP, PWM etc
    uint8_t jointEnable;
    uint16_t outputs;
    uint8_t spare0;
  };

  rxData_t() {
      header = 0;
      outputs = 0;
      jointEnable = 0;
      for (int i=0;i<JOINTS;i++) {
         jointFreqCmd[i] = 0;
      }
      for (int i=0;i<VARIABLES;i++) {
         setPoint[i] = 0.0;
     }
  }
} rxData_t;

typedef union txData_t
{
  // this allow structured access to the out going SPI data without having to move it
  struct
  {
    uint8_t txBuffer[DATA_BUFF_SIZE];
  };
  struct
  {
    uint32_t header;
    uint32_t jointFeedback[JOINTS];	  // Base thread feedback ??
    float processVariable[VARIABLES];		     // Servo thread feedback ??
	uint16_t inputs;
  };

  txData_t() {
      header = 0;
      inputs = 0;
      for (int i=0;i<JOINTS;i++) {
         jointFeedback[i] = 0;
      }
      for (int i=0;i<VARIABLES;i++) {
         processVariable[i] = 0.0;
     }
  }
} txData_t;

#pragma pack(pop)

// boolean
static volatile bool PRUreset;
// Global Data Buffers
extern volatile rxData_t* rxData;
extern volatile txData_t* txData;
// pointers to data
extern volatile uint32_t* ptrTxHeader;
extern volatile bool*    ptrPRUreset;
extern volatile uint32_t* ptrJointFreqCmd;
extern volatile uint32_t* ptrJointFeedback;
extern volatile uint8_t* ptrJointEnable;
extern volatile float*   ptrSetPoint;
extern volatile float*   ptrProcessVariable;
extern volatile uint16_t* ptrInputs;
extern volatile uint16_t* ptrOutputs;

extern RemoraCAN* comms;

#endif
