/********************************************************************
* Description:  remora.c
*               Modified version using WiringPi for hardware interface
*               HAL component that provides SPI connection to external 
*               STM32 running Remora PRU firmware.
*
* Author: Original by Scott Alford, WiringPi modifications added
* License: GPL Version 2
********************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include "remora.h"

#define MODNAME "remora"
#define PREFIX "remora"
#define author "Scott Alford AKA scotta";
#define description "Driver for Remora STM32 control board";
#define license "GPL v2";
MODULE_AUTHOR(author);
MODULE_DESCRIPTION(description);
MODULE_LICENSE(license);

// SPI settings
#define SPI_CHANNEL 0
#define SPI_SPEED 32000000  // 32MHz max for RPi SPI
                            //

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

typedef struct {
	hal_bit_t		*SPIenable;
	hal_bit_t		*SPIreset;
	hal_bit_t		*PRUreset;
	bool			SPIresetOld;
	hal_bit_t		*SPIstatus;
	hal_bit_t 		*stepperEnable[JOINTS];
	int				pos_mode[JOINTS];
	hal_float_t 	*pos_cmd[JOINTS];			// pin: position command (position units)
	hal_float_t 	*vel_cmd[JOINTS];			// pin: velocity command (position units/sec)
	hal_float_t 	*pos_fb[JOINTS];			// pin: position feedback (position units)
	hal_s32_t		*count[JOINTS];				// pin: psition feedback (raw counts)
	hal_float_t 	pos_scale[JOINTS];			// param: steps per position unit
	float 			freq[JOINTS];				// param: frequency command sent to PRU
	hal_float_t 	*freq_cmd[JOINTS];			// pin: frequency command monitoring, available in LinuxCNC
	hal_float_t 	maxvel[JOINTS];				// param: max velocity, (pos units/sec)
	hal_float_t 	maxaccel[JOINTS];			// param: max accel (pos units/sec^2)
	hal_float_t		*pgain[JOINTS];
	hal_float_t		*ff1gain[JOINTS];
	hal_float_t		*deadband[JOINTS];
	float 			old_pos_cmd[JOINTS];		// previous position command (counts)
	float 			old_pos_cmd_raw[JOINTS];		// previous position command (counts)
	float 			old_scale[JOINTS];			// stored scale value
	float 			scale_recip[JOINTS];		// reciprocal value used for scaling
	float			prev_cmd[JOINTS];
	float			cmd_d[JOINTS];					// command derivative
	hal_float_t 	*setPoint[VARIABLES];
	hal_float_t 	*processVariable[VARIABLES];
	hal_bit_t   	*outputs[DIGITAL_OUTPUTS];
	hal_bit_t   	*inputs[DIGITAL_INPUTS];
} data_t;

static data_t *data;


#pragma pack(push, 1)

typedef union
{
  // this allow structured access to the outgoing SPI data without having to move it
  // this is the same structure as the PRU rxData structure
  struct
  {
    uint8_t txBuffer[SPIBUFSIZE];
  };
  struct
  {
	int32_t header;
    int32_t jointFreqCmd[JOINTS];
    float 	setPoint[VARIABLES];
	uint8_t jointEnable;
	uint16_t outputs;
    uint8_t spare0;
  };
} txData_t;


typedef union
{
  // this allow structured access to the incoming SPI data without having to move it
  // this is the same structure as the PRU txData structure
  struct
  {
    uint8_t rxBuffer[SPIBUFSIZE];
  };
  struct
  {
    int32_t header;
    int32_t jointFeedback[JOINTS];
    float 	processVariable[VARIABLES];
    uint16_t inputs;
  };
} rxData_t;

#pragma pack(pop)

#define ctrl_type_desc "control type (pos or vel)"
#define pru_chip_desc "PRU chip type; LPC or STM"
#define spi_desc "SPI clock divider"
#define pru_desc "PRU base thread frequency"

static txData_t txData;
static rxData_t rxData;


/* other globals */
static int 			comp_id;				// component ID
static const char 	*modname = MODNAME;
static const char 	*prefix = PREFIX;
static int 			num_chan = 0;			// number of step generators configured
static long 		old_dtns;				// update_freq function period in nsec - (THIS IS RUNNING IN THE PI)
static double		dt;						// update_freq period in seconds  - (THIS IS RUNNING IN THE PI)
static double 		recip_dt;				// recprocal of period, avoids divides

static int64_t 		accum[JOINTS] = { 0 };
static int32_t 		old_count[JOINTS] = { 0 };
static int32_t		accum_diff = 0;

static int 			reset_gpio_pin = 25;				// RPI GPIO pin number used to force watchdog reset of the PRU 

typedef enum CONTROL { POSITION, VELOCITY, INVALID } CONTROL;
char *ctrl_type[JOINTS] = { "p" };

RTAPI_MP_ARRAY_STRING(ctrl_type,JOINTS,ctrl_type_desc);

enum CHIP { LPC, STM } chip;
char *chip_type = { "STM" }; //default to STM
RTAPI_MP_STRING(chip_type, pru_chip_desc);

int SPI_clk_div = 32;
RTAPI_MP_INT(SPI_clk_div, spi_desc);

int PRU_base_freq = -1;
RTAPI_MP_INT(PRU_base_freq, pru_desc);


/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static void update_freq(void *arg, long period);
static void spi_write();
static void spi_read();
static void spi_transfer();
static CONTROL parse_ctrl_type(const char *ctrl);



static int rt_wiringpi_init(void) {
    // Initialize WiringPi
    if (wiringPiSetupGpio() == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to initialize WiringPi\n");
        return -1;
    }

    // Setup SPI
    if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "Failed to initialize SPI\n");
        return -1;
    }

    // Configure GPIO for PRU reset pin
    pinMode(reset_gpio_pin, OUTPUT);
    digitalWrite(reset_gpio_pin, LOW);

    return 1;
}

void spi_transfer() {
    if (chip == LPC) {
        // For LPC, transfer one byte at a time
        for (int i = 0; i < SPIBUFSIZE; i++) {
            unsigned char buf[1];
            buf[0] = txData.txBuffer[i];
            wiringPiSPIDataRW(SPI_CHANNEL, buf, 1);
            rxData.rxBuffer[i] = buf[0];
        }
    }
    else if (chip == STM) {
        // For STM, transfer entire buffer at once
        wiringPiSPIDataRW(SPI_CHANNEL, (unsigned char *)txData.txBuffer, SPIBUFSIZE);
        memcpy(rxData.rxBuffer, txData.txBuffer, SPIBUFSIZE);
    }
}

void spi_read() {
    int i;
    double curr_pos;

    // Data header
    txData.header = PRU_READ;
    
    // Update PRU reset output using WiringPi
    digitalWrite(reset_gpio_pin, *(data->PRUreset) ? HIGH : LOW);
    
    if (*(data->SPIenable)) {
        if ((*(data->SPIreset) && !(data->SPIresetOld)) || *(data->SPIstatus)) {
            // Reset rising edge detected or PRU running
            spi_transfer();

            switch (rxData.header) {
                case PRU_DATA:
                    // Process valid PRU data
                    *(data->SPIstatus) = 1;

                    for (i = 0; i < JOINTS; i++) {
                        // Convert PRU DDS accumulator (32 bit) to 64 bits
                        accum_diff = rxData.jointFeedback[i] - old_count[i];
                        old_count[i] = rxData.jointFeedback[i];
                        accum[i] += accum_diff;

                        *(data->count[i]) = accum[i] >> STEPBIT;

                        data->scale_recip[i] = (1.0 / STEP_MASK) / data->pos_scale[i];
                        curr_pos = (double)(accum[i]-STEP_OFFSET) * (1.0 / STEP_MASK);
                        *(data->pos_fb[i]) = (float)((curr_pos+0.5) / data->pos_scale[i]);
                    }

                    // Process feedback variables
                    for (i = 0; i < VARIABLES; i++) {
                        *(data->processVariable[i]) = rxData.processVariable[i]; 
                    }

                    // Process digital inputs
                    for (i = 0; i < DIGITAL_INPUTS; i++) {
                        *(data->inputs[i]) = (rxData.inputs & (1 << i)) ? 1 : 0;
                    }
                    break;
                    
                case PRU_ESTOP:
                    *(data->SPIstatus) = 0;
                    rtapi_print_msg(RTAPI_MSG_ERR, "An E-stop is active");
                    break;

                default:
                    *(data->SPIstatus) = 0;
                    rtapi_print("Bad SPI payload = %x\n", rxData.header);
                    break;
            }
        }
    }
    else {
        *(data->SPIstatus) = 0;
    }
    
    data->SPIresetOld = *(data->SPIreset);
}

void spi_write() {
    int i;

    txData.header = PRU_WRITE;

    // Set joint frequency commands
    for (i = 0; i < JOINTS; i++) {
        txData.jointFreqCmd[i] = data->freq[i];
    }

    // Set joint enable bits
    txData.jointEnable = 0;
    for (i = 0; i < JOINTS; i++) {
        if (*(data->stepperEnable[i])) {
            txData.jointEnable |= (1 << i);
        }
    }

    // Set process variables
    for (i = 0; i < VARIABLES; i++) {
        txData.setPoint[i] = *(data->setPoint[i]);
    }

    // Set digital outputs
    txData.outputs = 0;
    for (i = 0; i < DIGITAL_OUTPUTS; i++) {
        if (*(data->outputs[i])) {
            txData.outputs |= (1 << i);
        }
    }

    if (*(data->SPIstatus)) {
        spi_transfer();
    }
}

int rtapi_app_main(void) {
    char name[HAL_NAME_LEN + 1];
    int n, retval;

    // Parse control types for each joint
    for (n = 0; n < JOINTS; n++) {
        if (parse_ctrl_type(ctrl_type[n]) == INVALID) {
            rtapi_print_msg(RTAPI_MSG_ERR,
                    "STEPGEN: ERROR: bad control type '%s' for axis %i (must be 'p' or 'v')\n",
                    ctrl_type[n], n);
            return -1;
        }
    }

    // Validate chip type
    if (!strcmp(chip_type, "LPC") || !strcmp(chip_type, "lpc")) {
        rtapi_print_msg(RTAPI_MSG_INFO, "PRU: Chip type set to LPC\n");
        chip = LPC;
    }
    else if (!strcmp(chip_type, "STM") || !strcmp(chip_type, "stm")) {
        rtapi_print_msg(RTAPI_MSG_INFO, "PRU: Chip type set to STM\n");
        chip = STM;
    }
    else {
        rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: PRU chip type (must be 'LPC' or 'STM')\n");
        return -1;
    }

    // Validate PRU base frequency
    if (PRU_base_freq != -1) {
        if (PRU_base_freq < 40000 || PRU_base_freq > 240000) {
            rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: PRU base frequency incorrect\n");
            return -1;
        }
    }
    else {
        PRU_base_freq = PRU_BASEFREQ;
    }

    // Initialize HAL
    comp_id = hal_init(modname);
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed\n", modname);
        return -1;
    }

    // Allocate HAL shared memory
    data = hal_malloc(sizeof(data_t));
    if (!data) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", modname);
        hal_exit(comp_id);
        return -1;
    }

    // Initialize WiringPi and SPI
    if (!rt_wiringpi_init()) {
        rtapi_print_msg(RTAPI_MSG_ERR, "WiringPi initialization failed\n");
        hal_exit(comp_id);
        return -1;
    }

    // Export HAL pins and parameters
    // ... [rest of pin/parameter export code remains the same]

    // Export functions
    rtapi_snprintf(name, sizeof(name), "%s.update-freq", prefix);
    retval = hal_export_funct(name, update_freq, data, 1, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: update function export failed\n", modname);
        hal_exit(comp_id);
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.write", prefix);
    retval = hal_export_funct(name, spi_write, 0, 0, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: write function export failed\n", modname);
        hal_exit(comp_id);
        return -1;
    }

    rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
    retval = hal_export_funct(name, spi_read, data, 1, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: read function export failed\n", modname);
        hal_exit(comp_id);
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
    hal_ready(comp_id);
    return 0;
}

void update_freq(void *arg, long period)
{
	int i;
	data_t *data = (data_t *)arg;
	double max_ac, vel_cmd, dv, new_vel, max_freq, desired_freq;
		   
	double error, command, feedback;
	double periodfp, periodrecip;
	float pgain, ff1gain, deadband;

	// precalculate timing constants
    periodfp = period * 0.000000001;
    periodrecip = 1.0 / periodfp;

    // calc constants related to the period of this function (LinuxCNC SERVO_THREAD)
    // only recalc constants if period changes
    if (period != old_dtns) 			// Note!! period = LinuxCNC SERVO_PERIOD
	{
		old_dtns = period;				// get ready to detect future period changes
		dt = period * 0.000000001; 		// dt is the period of this thread, used for the position loop
		recip_dt = 1.0 / dt;			// calc the reciprocal once here, to avoid multiple divides later
    }

    // loop through generators
	for (i = 0; i < JOINTS; i++)
	{
		// check for scale change
		if (data->pos_scale[i] != data->old_scale[i])
		{
			data->old_scale[i] = data->pos_scale[i];		// get ready to detect future scale changes
			// scale must not be 0
			if ((data->pos_scale[i] < 1e-20) && (data->pos_scale[i] > -1e-20))	// validate the new scale value
				data->pos_scale[i] = 1.0;										// value too small, divide by zero is a bad thing
				// we will need the reciprocal, and the accum is fixed point with
				//fractional bits, so we precalc some stuff
			data->scale_recip[i] = (1.0 / STEP_MASK) / data->pos_scale[i];
		}

		// calculate frequency limit
		//max_freq = PRU_BASEFREQ/(4.0); 			//limit of DDS running at 80kHz
		//max_freq = PRU_BASEFREQ/(2.0); 	
		max_freq = PRU_base_freq/(2.0);

		// check for user specified frequency limit parameter
		if (data->maxvel[i] <= 0.0)
		{
			// set to zero if negative
			data->maxvel[i] = 0.0;
		}
		else
		{
			// parameter is non-zero, compare to max_freq
			desired_freq = data->maxvel[i] * fabs(data->pos_scale[i]);

			if (desired_freq > max_freq)
			{
				// parameter is too high, limit it
				data->maxvel[i] = max_freq / fabs(data->pos_scale[i]);
			}
			else
			{
				// lower max_freq to match parameter
				max_freq = data->maxvel[i] * fabs(data->pos_scale[i]);
			}
		}
		
		/* set internal accel limit to its absolute max, which is
		zero to full speed in one thread period */
		max_ac = max_freq * recip_dt;
		
		// check for user specified accel limit parameter
		if (data->maxaccel[i] <= 0.0)
		{
			// set to zero if negative
			data->maxaccel[i] = 0.0;
		}
		else 
		{
			// parameter is non-zero, compare to max_ac
			if ((data->maxaccel[i] * fabs(data->pos_scale[i])) > max_ac)
			{
				// parameter is too high, lower it
				data->maxaccel[i] = max_ac / fabs(data->pos_scale[i]);
			}
			else
			{
				// lower limit to match parameter
				max_ac = data->maxaccel[i] * fabs(data->pos_scale[i]);
			}
		}

		/* at this point, all scaling, limits, and other parameter
		changes have been handled - time for the main control */

		

		if (data->pos_mode[i]) {

			/* POSITION CONTROL MODE */

			// use Proportional control with feed forward (pgain, ff1gain and deadband)
			
			if (*(data->pgain[i]) != 0)
			{
				pgain = *(data->pgain[i]);
			}
			else
			{
				pgain = 1.0;
			}
			
			if (*(data->ff1gain[i]) != 0)
			{
				ff1gain = *(data->ff1gain[i]);
			}
			else
			{
				ff1gain = 1.0;
			}
			
			if (*(data->deadband[i]) != 0)
			{
				deadband = *(data->deadband[i]);
			}
			else
			{
				deadband = 1 / data->pos_scale[i];
			}	

			// read the command and feedback
			command = *(data->pos_cmd[i]);
			feedback = *(data->pos_fb[i]);
			
			// calcuate the error
			error = command - feedback;
			
			// apply the deadband
			if (error > deadband)
			{
				error -= deadband;
			}
			else if (error < -deadband)
			{
				error += deadband;
			}
			else
			{
				error = 0;
			}
			
			// calcuate command and derivatives
			data->cmd_d[i] = (command - data->prev_cmd[i]) * periodrecip;
			
			// save old values
			data->prev_cmd[i] = command;
				
			// calculate the output value
			vel_cmd = pgain * error + data->cmd_d[i] * ff1gain;
		
		} else {

			/* VELOCITY CONTROL MODE */
			
			// calculate velocity command in counts/sec
			vel_cmd = *(data->vel_cmd[i]);
		}	
			
		vel_cmd = vel_cmd * data->pos_scale[i];
			
		// apply frequency limit
		if (vel_cmd > max_freq) 
		{
			vel_cmd = max_freq;
		} 
		else if (vel_cmd < -max_freq) 
		{
			vel_cmd = -max_freq;
		}
		
		// calc max change in frequency in one period
		dv = max_ac * dt;
		
		// apply accel limit
		if ( vel_cmd > (data->freq[i] + dv) )
		{
			new_vel = data->freq[i] + dv;
		} 
		else if ( vel_cmd < (data->freq[i] - dv) ) 
		{
			new_vel = data->freq[i] - dv;
		}
		else
		{
			new_vel = vel_cmd;
		}
		
		// test for disabled stepgen
		if (*data->stepperEnable == 0) {
			// set velocity to zero
			new_vel = 0; 
		}
		
		data->freq[i] = new_vel;				// to be sent to the PRU
		*(data->freq_cmd[i]) = data->freq[i];	// feedback to LinuxCNC
	}

}


static CONTROL parse_ctrl_type(const char *ctrl)
{
    if(!ctrl || !*ctrl || *ctrl == 'p' || *ctrl == 'P') return POSITION;
    if(*ctrl == 'v' || *ctrl == 'V') return VELOCITY;
    return INVALID;
}


void rtapi_app_exit(void) {
    // Close SPI
    close(wiringPiSPIGetFd(SPI_CHANNEL));
    hal_exit(comp_id);
}
