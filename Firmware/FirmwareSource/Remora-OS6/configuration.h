#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define ENABLE_REMORA_COMMS_SPI 0

#define CAN_ENABLE          1
#define PRU_BASEFREQ    	500 //24000   // PRU Base thread ISR update frequency (hz)
#define PRU_SERVOFREQ       500            // PRU Servo thread ISR update freqency (hz)
#define COMM_FREQ           500            // PRU Servo thread ISR update freqency (hz)
#define OVERSAMPLE          3
#define SWBAUDRATE          19200           // Software serial baud rate
#define PRU_COMMSFREQ       (SWBAUDRATE * OVERSAMPLE)

#define STEPBIT     		22            	// bit location in DDS accum
#define STEP_MASK   		(1L<<STEPBIT)

#define JSON_BUFF_SIZE	    10000			// Jason dynamic buffer size

#define JOINTS			    3				// Number of joints - set this the same as LinuxCNC HAL compenent. Max 8 joints
#define VARIABLES           6             	// Number of command values - set this the same as the LinuxCNC HAL compenent

#define PRU_DATA		    0x64617461 	    // "data" SPI payload
#define PRU_READ            0x72656164      // "read" SPI payload
#define PRU_WRITE           0x77726974      // "writ" SPI payload
#define PRU_ESTOP           0x65737470      // "estp" SPI payload


// Serial configuration
#define TXD0                P0_2            // MBED pin number
#define RXD0                P0_3
#define PC_BAUD             115200          // UART baudrate


#define LOOP_TIME           .1
#define SPI_ERR_MAX         5
// PRU reset will occur in SPI_ERR_MAX * LOOP_TIME = 0.5sec

#define SPI_BUFF_SIZE      64

#define DATA_BUFF_SIZE                 ((JOINTS * 4) + (VARIABLES * 4) + 6)    // total size of JOINTS + VARIABLES + HEADER + OUTPUTS, rxData is 2 larger then txData
                                                        //

#define MOSI0               P0_18           // RPi SPI
#define MISO0               P0_17
#define SCK0                P0_15
#define SSEL0               P0_16

#define WATCHDOG_TIMEOUT    5000
//#define MOSI1               P0_9            // SD card
//#define MISO1               P0_8
//#define SCK1                P0_7
//#define SSEL1               P0_6

#if defined TARGET_SKRV3
#define CAN_TX PB_9
#define CAN_RX PB_8
#elif defined TARGET_OCTOPUS_446 || defined TARGET_OCTOPUS_723
#define CAN_RX PD_0
#define CAN_TX PD_1
#endif



#endif
