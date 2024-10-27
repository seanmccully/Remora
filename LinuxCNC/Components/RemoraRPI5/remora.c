#include "remora.h"
#include "remora_log.h"


MODULE_AUTHOR("Scott Alford - Modified for CAN");
MODULE_DESCRIPTION("Driver for Remora CAN control board");
MODULE_LICENSE("GPL v2");

// Packed data structures for CAN transfer
#pragma pack(push, 1)
typedef union {
    struct {
        uint8_t txBuffer[CAN_BUFF_SIZE];
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
        uint8_t rxBuffer[CAN_BUFF_SIZE];
    };
    struct {
        int32_t header;
        int32_t jointFeedback[JOINTS];
        float   processVariable[VARIABLES];
        uint16_t inputs;
    };
} rxData_t;
#pragma pack(pop)

typedef struct {
    hal_bit_t        *CANenable;
    hal_bit_t        *CANreset;
    hal_bit_t        *PRUreset;
    bool             CANresetOld;
    hal_bit_t        *CANstatus;
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

/* other globals */
uint8_t spi_fd = 0;
static txData_t txData;
static rxData_t rxData;
static int comp_id;
static data_t *data;
static int64_t old_dtns;
static double dt;
static double recip_dt;
static int32_t old_count[JOINTS];
static int64_t accum[JOINTS];
static bool PRU_reset_state = false;
static int can_socket = -1;

static const char 	*modname = MODNAME;
static const char 	*prefix = PREFIX;
static long 		old_dtns;				// update_freq function period in nsec - (THIS IS RUNNING IN THE PI)
static double		dt;						// update_freq period in seconds  - (THIS IS RUNNING IN THE PI)
static double 		recip_dt;				// recprocal of period, avoids divides

static int64_t 		accum[JOINTS] = { 0 };
static int32_t 		old_count[JOINTS] = { 0 };

// Function declarations
static int can_init(void);
static void can_cleanup(void);
static int can_send(uint32_t can_id, uint8_t *data, uint8_t len);
static int can_receive(uint32_t *can_id, uint8_t *data, uint8_t *len);
static void update_freq(void *arg, long period);
static void can_write(void *arg, long period);
static void can_read(void *arg, long period);
static void spi_cleanup(void);
static int hal_err_handle(int _ret);
static CONTROL parse_ctrl_type(const char *ctrl);
void rtapi_app_exit(void);

// CAN message creation helper
static uint32_t make_can_id(uint8_t priority, uint8_t src, uint8_t dest, uint8_t type, uint8_t seq) {
    return (((uint32_t)priority & 0x7) << 26) |
           (((uint32_t)src & 0x1F) << 21) |
           (((uint32_t)dest & 0x1F) << 16) |
           (((uint32_t)type & 0xFF) << 8) |
           ((uint32_t)seq & 0xFF);
}

static int can_init(void) {
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) {
        log_error("%s: ERROR: Cannot create CAN socket: %s\n", MODNAME, strerror(errno));
        return -1;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, DEFAULT_CAN_INTERFACE);
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        log_error("%s: ERROR: Cannot get CAN interface index: %s\n", MODNAME, strerror(errno));
        close(can_socket);
        return -1;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error("%s: ERROR: Cannot bind CAN socket: %s\n", MODNAME, strerror(errno));
        close(can_socket);
        return -1;
    }

    return 0;
}

static int can_send(uint32_t can_id, uint8_t *data, uint8_t len) {
    struct can_frame frame = {0};
    frame.can_id = can_id;
    frame.can_dlc = len;
    memcpy(frame.data, data, len);
    
    int result = write(can_socket, &frame, sizeof(struct can_frame));
    if (result != sizeof(struct can_frame)) {
        log_error("%s: ERROR: CAN write failed: %s\n", MODNAME, strerror(errno));
        return -1;
    }
    return 0;
}

static int can_receive(uint32_t *can_id, uint8_t *data, uint8_t *len) {
    struct can_frame frame;
    int result = read(can_socket, &frame, sizeof(struct can_frame));
    
    if (result < 0) {
        if (errno != EAGAIN) {
            log_error("%s: ERROR: CAN read failed: %s\n", MODNAME, strerror(errno));
        }
        return -1;
    }
    
    *can_id = frame.can_id;
    *len = frame.can_dlc;
    memcpy(data, frame.data, frame.can_dlc);
    return 0;
}

static void can_write(void *arg, long period) {
    data_t *data = (data_t *)arg;
    uint8_t joint_enables = 0;

    if (!*(data->CANenable) || !*(data->CANstatus)) {
        return;
    }

    // Send joint commands and collect enables
    for (int i = 0; i < JOINTS; i++) {
        uint32_t can_id = make_can_id(PRIO_MEDIUM, NODE_CONTROLLER, NODE_REMORA, MSG_JOINT_CMD, i);
        float freq = data->freq[i];
        can_send(can_id, (uint8_t*)&freq, sizeof(float));
        
        if (*(data->stepperEnable[i])) {
            joint_enables |= (1 << i);
        }
    }

    // Send setpoints
    for (int i = 0; i < VARIABLES; i++) {
        uint32_t can_id = make_can_id(PRIO_MEDIUM, NODE_CONTROLLER, NODE_REMORA, MSG_SETPOINT, i);
        float setpoint = *(data->setPoint[i]);
        can_send(can_id, (uint8_t*)&setpoint, sizeof(float));
    }

    // Send digital outputs
    for (int i = 0; i < (DIGITAL_OUTPUTS + 7) / 8; i++) {
        uint32_t can_id = make_can_id(PRIO_LOW, NODE_CONTROLLER, NODE_REMORA, MSG_DIG_OUT, i);
        uint8_t out_byte = 0;
        for (int bit = 0; bit < 8 && (i * 8 + bit) < DIGITAL_OUTPUTS; bit++) {
            if (*(data->outputs[i * 8 + bit])) {
                out_byte |= (1 << bit);
            }
        }
        can_send(can_id, &out_byte, 1);
    }

    // Send enables status
    uint32_t enable_id = make_can_id(PRIO_HIGH, NODE_CONTROLLER, NODE_REMORA, MSG_STATUS, 0);
    can_send(enable_id, &joint_enables, 1);
}

static void can_read(void *arg, long period) {
    data_t *data = (data_t *)arg;
    uint32_t can_id;
    uint8_t rx_data[8];
    uint8_t rx_len;
    

    while (can_receive(&can_id, rx_data, &rx_len) == 0) {
        uint8_t msg_type = (can_id >> 8) & 0xFF;
        uint8_t index = can_id & 0xFF;

        switch (msg_type) {
            case MSG_JOINT_FB: {
                *(data->CANstatus) = true;
                if (index < JOINTS) {
                    int32_t counts;
                    memcpy(&counts, rx_data, sizeof(int32_t));
                    
                    // Update position feedback
                    int32_t diff = counts - old_count[index];
                    old_count[index] = counts;
                    accum[index] += diff;
                    
                    *(data->count[index]) = counts;
                    if (data->pos_scale[index] != 0.0) {
                        *(data->pos_fb[index]) = counts / data->pos_scale[index];
                    }
                }
                break;
            }

            case MSG_PV_FB: {
                *(data->CANstatus) = true;
                if (index < VARIABLES && rx_len == sizeof(float)) {
                    float value;
                    memcpy(&value, rx_data, sizeof(float));
                    *(data->processVariable[index]) = value;
                }
                break;
            }

            case MSG_DIG_IN: {
                *(data->CANstatus) = true;
                int base_input = index * 8;
                for (int i = 0; i < 8 && base_input + i < DIGITAL_INPUTS; i++) {
                    *(data->inputs[base_input + i]) = (rx_data[0] & (1 << i)) ? 1 : 0;
                }
                break;
            }

            case MSG_STATUS: {
                *(data->CANstatus) = (rx_data[0] != 0);
                break;
            }
        }
    }
}

int rtapi_app_main(void) {
    int retval;

    comp_id = hal_init(MODNAME);
    if (comp_id < 0) {
        log_error("%s: ERROR: hal_init() failed\n", MODNAME);
        return -1;
    }

    // Allocate HAL shared memory
    data = hal_malloc(sizeof(data_t));
    if (!data) {
        log_error("%s: ERROR: hal_malloc() failed\n", MODNAME);
        hal_exit(comp_id);
        return -1;
    }

    // Initialize CAN interface
    if (can_init() < 0) {
        log_error("%s: ERROR: CAN initialization failed\n", MODNAME);
        hal_exit(comp_id);
        return -1;
    }

    // Export HAL pins and parameters
    retval = remora_hal_pins();
    if (retval < 0) goto error;
    
    retval = remora_hal_joint_pins();
    if (retval < 0) goto error;
    
    retval = remora_hal_joints();
    if (retval < 0) goto error;

    // Export HAL functions
    retval = hal_export_funct("remora.update-freq", update_freq, data, 1, 0, comp_id);
    if (retval < 0) goto error;

    retval = hal_export_funct("remora.write", can_write, data, 0, 0, comp_id);
    if (retval < 0) goto error;

    retval = hal_export_funct("remora.read", can_read, data, 1, 0, comp_id);
    if (retval < 0) goto error;

    hal_ready(comp_id);
    return 0;

error:
    can_cleanup();
    hal_exit(comp_id);
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
    int retval = hal_pin_bit_newf(HAL_IN, &(data->CANenable), comp_id, "%s.CAN-enable", prefix);
    if (retval != 0) return retval;
    
    retval = hal_pin_bit_newf(HAL_IN, &(data->CANreset), comp_id, "%s.CAN-reset", prefix);
    if (retval != 0) return retval;

    retval = hal_pin_bit_newf(HAL_OUT, &(data->CANstatus), comp_id, "%s.CAN-status", prefix);
    if (retval != 0) return retval;

    retval = hal_pin_bit_newf(HAL_IN, &(data->PRUreset), comp_id, "%s.PRU-reset", prefix);
    if (retval != 0) return retval;
    return retval;
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

static void spi_cleanup(void) {
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }
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

void rtapi_app_exit(void) {
    can_cleanup();
    hal_exit(comp_id);
}

static void can_cleanup(void) {
    if (can_socket >= 0) {
        close(can_socket);
        can_socket = -1;
    }
}

