/********************************************************************
* Description:  remora.c
*               This file, 'remora.c', is a HAL component that
*               provides an SPI connection to an external STM32 running Remora PRU firmware.
*               
*               Modified for Raspberry Pi 5 to use Linux kernel SPI interface
*               instead of bcm2835 direct hardware access.
*
* Author: Scott Alford
* License: GPL Version 2
*
* Copyright (c) 2021-2024 All rights reserved.
********************************************************************/

#include "rtapi.h"          /* RTAPI realtime OS API */
#include "rtapi_app.h"      /* RTAPI realtime module decls */
#include "hal.h"            /* HAL public API decls */

#include <math.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "remora.h"
#include "remora_log.h"

#define MODNAME "remora"
#define PREFIX "remora"

MODULE_AUTHOR("Scott Alford");
MODULE_DESCRIPTION("Driver for Remora STM32 control board - Modified for RPi 5");
MODULE_LICENSE("GPL v2");


typedef struct {
    hal_bit_t        *SPIenable;
    hal_bit_t        *SPIreset;
    hal_bit_t        *PRUreset;
    bool             SPIresetOld;
    hal_bit_t        *SPIstatus;
    hal_bit_t        *stepperEnable[JOINTS];
    int              pos_mode[JOINTS];
    hal_float_t      *pos_cmd[JOINTS];     /* pin: position command */
    hal_float_t      *vel_cmd[JOINTS];     /* pin: velocity command */
    hal_float_t      *pos_fb[JOINTS];      /* pin: position feedback */
    hal_s32_t        *count[JOINTS];       /* pin: position feedback (raw counts) */
    hal_float_t      pos_scale[JOINTS];    /* param: steps per position unit */
    float            freq[JOINTS];         /* param: frequency command sent to PRU */
    hal_float_t      *freq_cmd[JOINTS];    /* pin: frequency command monitoring */
    hal_float_t      maxvel[JOINTS];       /* param: max velocity */
    hal_float_t      maxaccel[JOINTS];     /* param: max accel */
    hal_float_t      *pgain[JOINTS];
    hal_float_t      *ff1gain[JOINTS];
    hal_float_t      *deadband[JOINTS];
    float            old_pos_cmd[JOINTS];  /* previous position command */
    float            old_pos_cmd_raw[JOINTS]; /* previous position command */
    float            old_scale[JOINTS];    /* stored scale value */
    float            scale_recip[JOINTS];  /* reciprocal value used for scaling */
    float            prev_cmd[JOINTS];
    float            cmd_d[JOINTS];        /* command derivative */
    hal_float_t      *setPoint[VARIABLES];
    hal_float_t      *processVariable[VARIABLES];
    hal_bit_t        *outputs[DIGITAL_OUTPUTS];
    hal_bit_t        *inputs[DIGITAL_INPUTS];
} data_t;

static data_t *data;

/* Packed structs for SPI transfer */
#pragma pack(push, 1)

typedef union {
    struct {
        uint8_t txBuffer[SPIBUFSIZE];
    };
    struct {
        int32_t header;
        int32_t jointFreqCmd[JOINTS];
        float   setPoint[VARIABLES];
        uint8_t jointEnable;
        uint16_t outputs;
        uint8_t spare0;
    };
} txData_t;

typedef union {
    struct {
        uint8_t rxBuffer[SPIBUFSIZE];
    };
    struct {
        int32_t header;
        int32_t jointFeedback[JOINTS];
        float   processVariable[VARIABLES];
        uint16_t inputs;
    };
} rxData_t;

#pragma pack(pop)

static txData_t txData;
static rxData_t rxData;


// SPI settings
static const char *spi_device = "/dev/spidev0.0";  /* Default SPI device */
static int spi_fd = -1;
static uint32_t spi_speed = 32000000;  // Default 32MHz
static uint8_t spi_mode = SPI_MODE_0;
static uint8_t spi_bits = 8;


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
RTAPI_MP_ARRAY_STRING(ctrl_type,JOINTS,"control type (pos or vel)");

enum CHIP { LPC, STM } chip;
char *chip_type = { "STM" }; //default to STM
RTAPI_MP_STRING(chip_type, "PRU chip type; LPC or STM");

int SPI_clk_div = 32;
RTAPI_MP_INT(SPI_clk_div, "SPI clock divider");

int PRU_base_freq = -1;
RTAPI_MP_INT(PRU_base_freq, "PRU base thread frequency");


/* Function prototypes */
static int spi_init(void);
static void spi_cleanup(void);
static int spi_transfer(uint8_t *tx, uint8_t *rx, int len);
static void update_freq(void *arg, long period);
static void spi_write(void);
static void spi_read(void);
static int hal_err_handle(int _ret);
static int parse_stepgen(void);
static int validate_chip_type(void);
static int remora_hal_init(void);
static int remora_hal_pins(void);
static int remora_hal_joints(void);
static int remora_hal_joint_pins(void);
static int hal_error(void);
static int hal_export_functions(void);
static CONTROL parse_ctrl_type(const char *ctrl);
const char * concat(const char* pre,const char *post);


int rtapi_app_main(void) {
    int retval;
    int n;
    retval = parse_stepgen();
    retval = validate_chip_type();
    /* Initialize the SPI interface */
    if (spi_init() < 0) {
        log_error( "%s: ERROR: Failed to initialize SPI\n", modname);
        return -1;
    }

    /* Connect to the HAL */
    retval = remora_hal_init();

    /* Export pins and parameters */
    retval = remora_hal_pins();
    retval = remora_hal_joint_pins();
    retval = remora_hal_joints();

    /* ... Rest of pin/parameter exports ... */

    /* Export functions */
    retval = hal_export_functions();
    hal_ready(comp_id);
    return 0;

}

int parse_stepgen(void) {

	// parse stepgen control type
	for (int n = 0; n < JOINTS; n++) {
        if(parse_ctrl_type(ctrl_type[n]) == INVALID) {
          rtapi_print_msg(RTAPI_MSG_ERR,
              "STEPGEN: ERROR: bad control type '%s' for axis %i (must be 'p' or 'v')\n",
              ctrl_type[n], n);
          return -1;
        }
    }
}

int validate_chip_type(void) {
    // Validate chip type
    if (!strcmp(chip_type, "LPC") || !strcmp(chip_type, "lpc")) {
        log_info("PRU: Chip type set to LPC\n");
        chip = LPC;
    }
    else if (!strcmp(chip_type, "STM") || !strcmp(chip_type, "stm")) {
        log_info("PRU: Chip type set to STM\n");
        chip = STM;
    }
    else {
        log_error("ERROR: PRU chip type (must be 'LPC' or 'STM')\n");
        return -1;
    }
    return 0;
}

const char * concat(const char* pre,const char *post) {

    int pre_l = strlen(pre);
    int post_l = strlen(post);
    char* _str = malloc(pre_l + post_l);
    memcpy(_str,pre, pre_l);
    memcpy(_str,post, post_l+1);
    return _str;
}
int hal_export_functions(void) {

    int retval = hal_export_funct("remora.update-freq", update_freq, data, 1, 0, comp_id);
    if (retval < 0) {
        log_error( "%s: ERROR: update function export failed\n", modname);
        hal_exit(comp_id);
        return -1;
    }

    //rtapi_snprintf(name, strlen(name), "%s.write", prefix);
    retval = hal_export_funct("remora.write", spi_write, 0, 0, 0, comp_id);
    if (retval < 0) {
        log_error( "%s: ERROR: write function export failed\n", modname);
        hal_exit(comp_id);
        return -1;
    }

    //rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
    retval = hal_export_funct("remora.read", spi_read, data, 1, 0, comp_id);
    if (retval < 0) {
        log_error( "%s: ERROR: read function export failed\n", modname);
        hal_exit(comp_id);
        return -1;
    }

    log_info("%s: installed driver\n", modname);
    return retval;
}

int remora_hal_init(void) {

    comp_id = hal_init(modname);
    if (comp_id < 0) {
        log_error("%s ERROR: hal_init() failed\n", modname);
        return -1;
    }

    data = hal_malloc(sizeof(data_t));
    if (!data) {
        log_error("%s: ERROR: hal_malloc() failed\n", modname);
        hal_exit(comp_id);
        return -1;
    }
    return 0;
}

int remora_hal_pins(void) {

    // Export HAL pins and parameters
    int retval = hal_pin_bit_newf(HAL_IN, &(data->SPIenable), comp_id, "%s.SPI-enable", prefix);
    if (retval != 0) return retval;
    
    retval = hal_pin_bit_newf(HAL_IN, &(data->SPIreset), comp_id, "%s.SPI-reset", prefix);
    if (retval != 0) return retval;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->SPIstatus), comp_id, "%s.SPI-status", prefix);
    if (retval != 0) return retval;

    return 0;
    // Configure PRU reset pin
    //gpioSetMode(reset_gpio_pin, PI_OUTPUT);
    //retval = hal_pin_bit_newf(HAL_IN, &(data->PRUreset), comp_id, "%s.PRU-reset", prefix);
    //if (retval != 0) return retval;
    //return retval;
}

int remora_hal_joint_pins(void) {

  int retval=0;
	for (int n = 0; n < VARIABLES; n++) {
	// export pins

/*
This is throwing errors from axis.py for some reason...

		if (data->pos_mode[n]){
			log_error("Creating pos_mode[%d] = %d\n", n, data->pos_mode[n]);
			retval = hal_pin_float_newf(HAL_IN, &(data->pos_cmd[n]),
					comp_id, "%s.joint.%01d.pos-cmd", prefix, n);
			if (retval < 0) return hal_error();
			*(data->pos_cmd[n]) = 0.0;
		} else {
			log_error("Creating vel_mode[%d] = %d\n", n, data->pos_mode[n]);
			retval = hal_pin_float_newf(HAL_IN, &(data->vel_cmd[n]),
					comp_id, "%s.joint.%01d.vel-cmd", prefix, n);
			if (retval < 0) return hal_error();
			*(data->vel_cmd[n]) = 0.0;			
		}
*/
		retval = hal_pin_float_newf(HAL_IN, &(data->setPoint[n]),
		        comp_id, "%s.SP.%01d", prefix, n);
		hal_err_handle(retval);
		*(data->setPoint[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->processVariable[n]),
		        comp_id, "%s.PV.%01d", prefix, n);
		hal_err_handle(retval);
		*(data->processVariable[n]) = 0.0;
	}

	for (int n = 0; n < DIGITAL_OUTPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_IN, &(data->outputs[n]),
				comp_id, "%s.output.%01d", prefix, n);
		hal_err_handle(retval);
		*(data->outputs[n])=0;
	}

	for (int n = 0; n < DIGITAL_INPUTS; n++) {
		retval = hal_pin_bit_newf(HAL_OUT, &(data->inputs[n]),
				comp_id, "%s.input.%01d", prefix, n);
		hal_err_handle(retval);
		*(data->inputs[n])=0;
	}
  return retval;
}

int remora_hal_joints(void) {

    int retval = 0;
    for (int n = 0; n < JOINTS; n++) {

		    data->pos_mode[n] = (parse_ctrl_type(ctrl_type[n]) == POSITION);

        retval = hal_pin_bit_newf(HAL_IN, &(data->stepperEnable[n]),
            comp_id, "%s.joint.%01d.enable", prefix, n);
        hal_err_handle(retval);

        retval = hal_pin_float_newf(HAL_IN, &(data->pos_cmd[n]),
            comp_id, "%s.joint.%01d.pos-cmd", prefix, n);
        hal_err_handle(retval);
        *(data->pos_cmd[n]) = 0.0;
        
        if (data->pos_mode[n] == 0){
          retval = hal_pin_float_newf(HAL_IN, &(data->vel_cmd[n]),
              comp_id, "%s.joint.%01d.vel-cmd", prefix, n);
          hal_err_handle(retval);
          *(data->vel_cmd[n]) = 0.0;			
        }

        retval = hal_pin_float_newf(HAL_OUT, &(data->freq_cmd[n]),
                comp_id, "%s.joint.%01d.freq-cmd", prefix, n);
        hal_err_handle(retval);
        *(data->freq_cmd[n]) = 0.0;

        retval = hal_pin_float_newf(HAL_OUT, &(data->pos_fb[n]),
                comp_id, "%s.joint.%01d.pos-fb", prefix, n);
        hal_err_handle(retval);
        *(data->pos_fb[n]) = 0.0;
        
        retval = hal_param_float_newf(HAL_RW, &(data->pos_scale[n]),
                comp_id, "%s.joint.%01d.scale", prefix, n);
        hal_err_handle(retval);
        data->pos_scale[n] = 1.0;

        retval = hal_pin_s32_newf(HAL_OUT, &(data->count[n]),
                comp_id, "%s.joint.%01d.counts", prefix, n);
        hal_err_handle(retval);
        *(data->count[n]) = 0;
        
        retval = hal_pin_float_newf(HAL_IN, &(data->pgain[n]),
            comp_id, "%s.joint.%01d.pgain", prefix, n);
        hal_err_handle(retval);
        *(data->pgain[n]) = 0.0;
        
        retval = hal_pin_float_newf(HAL_IN, &(data->ff1gain[n]),
            comp_id, "%s.joint.%01d.ff1gain", prefix, n);
        hal_err_handle(retval);
        *(data->ff1gain[n]) = 0.0;
        
        retval = hal_pin_float_newf(HAL_IN, &(data->deadband[n]),
            comp_id, "%s.joint.%01d.deadband", prefix, n);
        hal_err_handle(retval);
        *(data->deadband[n]) = 0.0;
        
        retval = hal_param_float_newf(HAL_RW, &(data->maxaccel[n]),
                comp_id, "%s.joint.%01d.maxaccel", prefix, n);
        hal_err_handle(retval);
        data->maxaccel[n] = 1.0;
	}
    return retval;
}


void rtapi_app_exit(void) {
    spi_cleanup();
    hal_exit(comp_id);
}

static int spi_init(void) {
    int ret;
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 15000000; // 15MHz - adjust as needed

    spi_fd = open(spi_device, O_RDWR);
    if (spi_fd < 0) {
        log_error( "%s: ERROR: Can't open device %s: %s\n",
                       modname, spi_device, strerror(errno));
        return -1;
    }

    /* SPI mode */
    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    if (ret < 0) {
        log_error( "%s: ERROR: Can't set SPI mode: %s\n",
                       modname, strerror(errno));
        return -1;
    }

    /* Bits per word */
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret < 0) {
        log_error( "%s: ERROR: Can't set bits per word: %s\n",
                       modname, strerror(errno));
        return -1;
    }

    /* Max speed Hz */
    ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret < 0) {
        log_error( "%s: ERROR: Can't set max speed Hz: %s\n",
                       modname, strerror(errno));
        return -1;
    }

    return 0;
}

static void spi_cleanup(void) {
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }
}

static int spi_transfer(uint8_t *tx, uint8_t *rx, int len) {
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len,
        .delay_usecs = 0,
        .speed_hz = 15000000,
        .bits_per_word = 8,
    };

    return ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

/* Update frequency function - same as original */
static void update_freq(void *arg, long period)
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

/* SPI write function using Linux SPI interface */
static void spi_write() {
    int i=0;
    /* Data header */
    txData.header = PRU_WRITE;

    /* Joint frequency commands */
    for (i = 0; i < JOINTS; i++) {
        txData.jointFreqCmd[i] = data->freq[i];
    }

    /* Joint enable bits */
    for (i = 0; i < JOINTS; i++) {
        if (*(data->stepperEnable[i]) == 1) {
            txData.jointEnable |= (1 << i);
        } else {
            txData.jointEnable &= ~(1 << i);
        }
    }

    /* Set points */
    for (i = 0; i < VARIABLES; i++) {
        txData.setPoint[i] = *(data->setPoint[i]);
    }

    /* Outputs */
    for (i = 0; i < DIGITAL_OUTPUTS; i++) {
        if (*(data->outputs[i]) == 1) {
            txData.outputs |= (1 << i);
        } else {
            txData.outputs &= ~(1 << i);
        }
    }

    if (*(data->SPIstatus)) {
        spi_transfer(txData.txBuffer, rxData.rxBuffer, SPIBUFSIZE);
    }
}

/* SPI read function using Linux SPI interface */
static void spi_read() {
    int i;
    double curr_pos;

    /* Data header */
    txData.header = PRU_READ;

    if (*(data->SPIenable)) {
        if ((*(data->SPIreset) && !(data->SPIresetOld)) || *(data->SPIstatus)) {
            /* Transfer data */
            spi_transfer(txData.txBuffer, rxData.rxBuffer, SPIBUFSIZE);

            switch (rxData.header) {
                case PRU_DATA:
                    *(data->SPIstatus) = 1;

                    /* Process received data */
                    for (i = 0; i < JOINTS; i++) {
                        /* Position calculations */
                        accum_diff = rxData.jointFeedback[i] - old_count[i];
                        old_count[i] = rxData.jointFeedback[i];
                        accum[i] += accum_diff;

                        *(data->count[i]) = accum[i] >> STEPBIT;

                        data->scale_recip[i] = (1.0 / STEP_MASK) / data->pos_scale[i];
                        curr_pos = (double)(accum[i]-STEP_OFFSET) * (1.0 / STEP_MASK);
                        *(data->pos_fb[i]) = (float)((curr_pos+0.5) / data->pos_scale[i]);
                    }

                    /* Process variables */
                    for (i = 0; i < VARIABLES; i++) {
                        *(data->processVariable[i]) = rxData.processVariable[i];
                    }

                    /* Process inputs */
                    for (i = 0; i < DIGITAL_INPUTS; i++) {
                        *(data->inputs[i]) = (rxData.inputs & (1 << i)) ? 1 : 0;
                    }
                    break;

                case PRU_ESTOP:
                    *(data->SPIstatus) = 0;
                    log_error( "An E-stop is active\n");
                    break;

                default:
                    *(data->SPIstatus) = 0;
                    log_warning("Bad SPI payload = %x\n", rxData.header);
                    break;
            }
        }
    } else {
        *(data->SPIstatus) = 0;
    }

    data->SPIresetOld = *(data->SPIreset);
}

int hal_error(void) {
		hal_exit(comp_id);
		return -1;
}

static int hal_err_handle(int _ret) { 
    if (_ret < 0) { 
        return hal_error();
    }
    return 0; 
}

static CONTROL parse_ctrl_type(const char *ctrl)
{
    if(!ctrl || !*ctrl || *ctrl == 'p' || *ctrl == 'P') return POSITION;
    if(*ctrl == 'v' || *ctrl == 'V') return VELOCITY;
    return INVALID;
}
