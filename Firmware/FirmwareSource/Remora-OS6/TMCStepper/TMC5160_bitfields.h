#pragma once


namespace TMC5160_n {

  struct STATUS_t {
    union {
      uint32_t sr : 18;
      struct {
          uint8_t
          reset_flag       : 1,
          driver_error     : 1,
          sg2              : 1,
          standstill       : 1,
          velocity_reached : 1,
          position_reached : 1,
          status_stop_l    : 1,
          status_stop_r    : 1;
      };
    };
  };

  struct GCONF_t {
    constexpr static uint8_t address = 0x00;
    union {
      uint32_t sr : 18;
      struct {
          uint32_t  recalibrate       : 1,
                    faststandstill    : 1,
                    en_pwm_mode       : 1,
                    multistep_filt    : 1,
                    shaft             : 1,
                    diag0_error       : 1,
                    diag0_otpw        : 1,
                    diag0_stall       : 1,
                    diag1_stall       : 1,
                    diag1_index       : 1,
                    diag1_onstate     : 1,
                    diag1_steps_skipped : 1,
                    diag0_int_pushpull : 1,
                    diag1_poscomp_pushpull : 1,
                    small_hysteresis  : 1,
                    stop_enable       : 1,
                    direct_mode       : 1,
                    test_mode         : 1,
                    reserved          : 14; // Reserved bits
      };
    };
  };

  struct GSTAT_t {
    constexpr static uint8_t address = 0x01;
    union {
      uint8_t sr : 3;
      struct {
          bool reset    : 1,
               drv_err  : 1;
      };
    };
  };

  // IFCNT : R
  struct IFCNT_t {
    constexpr static uint8_t address = 0x02;
    union {
      uint8_t sr : 3;
      struct {
          uint32_t count       : 8,
                   reserved    : 24; // Reserved bits
      };
    };
  };

  struct SLAVECONF_t {
    constexpr static uint8_t address = 0x03;
    union {
      uint16_t sr : 12;
      struct {
          uint8_t slaveaddr : 8;
          uint8_t senddelay : 4;
      };
    };
  };

  struct IOIN_t {
    constexpr static uint8_t address = 0x04;
    union {
      uint32_t sr;
      struct {
          bool refl_step       : 1,
               reserved_1     : 1, // Reserved bit
               refl_dir        : 1,
               encb_den_cfg4   : 1,
               encb_den_cfg5   : 1,
               drv_enn         : 1,
               reserved_2     : 1, // Reserved bit
               encb_den_cfg6   : 1,
               swcomp_in       : 1;
          uint16_t reserved_3 : 14; // Reserved bits
          uint8_t version     : 8;
      };
    };
  };

  // OUTPUT : W
  struct OUTPUT_t {
    constexpr static uint8_t address = 0x05;
    union {
      uint32_t sr;
      struct {
          uint32_t io_polarity : 1,
                   version     : 31;
      };
    };
  };

  // X_COMPARE : W
  struct X_COMPARE_t {
    constexpr static uint8_t address = 0x06;
    union {
      uint32_t sr;
      struct {
          uint32_t x_compare;
      };
    };
  };

  // OTP_PROG : W
  struct OTP_PROG_t {
    union {
      uint32_t sr;
      struct {
          uint32_t otpbit    : 3,
                   otpbyte   : 2,
                   reserved  : 3, // Reserved bits
                   otpmagic  : 8,
                   reserved2 : 16; // Reserved bits
      };
    };
  };

  // OTP_READ : R
  struct OTP_READ_t {
    constexpr static uint8_t address = 0x07;
    union {
      uint32_t sr;
      struct {
          uint32_t otp0_0_4  : 5,
                   otp0_5    : 1,
                   otp0_6    : 1,
                   otp0_7    : 1,
                   reserved  : 24; // Reserved bits
      };
    };
  };

  // FACTORY_CONF : RW
  struct FACTORY_CONF_t {
    constexpr static uint8_t address = 0x08;
    union {
      uint32_t sr;
      struct {
          uint32_t fclktrim  : 5,
                   reserved : 27; // Reserved bits
      };
    };
  };

  // SHORT_CONF : W
  struct SHORT_CONF_t {
    constexpr static uint8_t address = 0x09;
    union {
      uint32_t sr;
      struct {
          uint32_t s2vs_level  : 4,
                   reserved_1 : 4, // Reserved bits
                   s2g_level   : 4,
                   reserved_2 : 4, // Reserved bits
                   shortfilter : 2,
                   shortdelay  : 1,
                   reserved_3 : 13; // Reserved bits
      };
    };
  };

  // DRV_CONF : W
  struct DRV_CONF_t {
    constexpr static uint8_t address = 0x0A;
    union {
      uint32_t sr;
      struct {
          uint32_t bbmtime     : 5,
                   reserved_1  : 3, // Reserved bits
                   bbmclks     : 4,
                   reserved_2  : 4, // Reserved bits
                   otselect    : 2,
                   drvstrenght : 2,
                   filt_isense : 2,
                   reserved_3  : 10; // Reserved bits
      };
    };
  };

  // GLOBAL_SCALER : W
  struct GLOBAL_SCALER_t {
    constexpr static uint8_t address = 0x0B;
    union {
      uint32_t sr;
      struct {
          uint32_t scaler    : 8,
                   reserved  : 24; // Reserved bits
      };
    };
  };

  // OFFSET_READ : R
  struct OFFSET_READ_t {
    constexpr static uint8_t address = 0x0C;
    union {
      uint32_t sr;
      struct {
          uint32_t phase_a  : 8,
                   phase_b  : 8,
                   reserved : 20; // Reserved bits
      };
    };
  };

  // IHOLD_IRUN : R
  struct IHOLD_IRUN_t {
    constexpr static uint8_t address = 0x10;
    union {
      uint8_t sr : 8;
      struct {
          uint32_t ihold      : 5,
                   reserved_1 : 3, // Reserved bits
                   irun       : 5,
                   reserved_2 : 3, // Reserved bits
                   iholddelay : 4,
                   reserved_3 : 12; // Reserved bits
      };
    };
  };

  struct TPOWERDOWN_t {
    constexpr static uint8_t address = 0x11;
    union {
      uint8_t sr : 8;
      struct {
          uint32_t tstep    : 8,
                   reserved : 24; // Reserved bits
      };
    };
  };

  // TSTEP : R
  struct TSTEP_t  {
    constexpr static uint8_t address = 0x12;
    union {
      uint8_t sr : 8;
      struct {
          uint32_t tstep    : 20,
                   reserved : 12; // Reserved bits
      };
    };
  };

  struct TPWMTHRS_t {
    constexpr static uint8_t address = 0x13;
    union {
      uint32_t sr : 20;
      struct {
          uint32_t tpwmthrs : 20,
                   reserved : 12; // Reserved bits
      };
    };
  };

  struct TCOOLTHRS_t {
    constexpr static uint8_t address = 0x14;
    union {
      uint32_t sr : 20;
      struct {
          uint32_t tcoolthrs : 20,
                   reserved  : 12; // Reserved bits
      };
    };
  };

  struct THIGH_t {
    constexpr static uint8_t address = 0x15;
    union {
      uint32_t sr : 20;
      struct {
          uint32_t thigh : 20,
                   reserved : 12; // Reserved bits
      };
    };
  };

  struct VDCMIN_t {
    constexpr static uint8_t address = 0x33;
    union {
      uint32_t sr : 23;
      struct {
          uint32_t vdcmin   : 23,
                   reserved : 9; // Reserved bits
      };
    };
  };

  // MSLUTn : W
  struct MSLUTn_t {
    constexpr static uint8_t address = 0x60;
    union {
      uint32_t sr : 32;
      struct {
          uint32_t mte : 32;
      };
    };
  };

  // MSLUTSEL : W
  struct MSLUTSEL_t {
    constexpr static uint8_t address = 0x68;
    union {
      uint32_t sr;
      struct {
          uint32_t w0 : 2,
                   w1 : 2,
                   w2 : 2,
                   w3 : 2,
                   x1 : 8,
                   x2 : 8,
                   x3 : 8;
      };
    };
  };

  // MSLUTSTART : W
  struct MSLUTSTART_t {
    constexpr static uint8_t address = 0x69;
    union {
      uint32_t sr;
      struct {
          uint32_t start_sin   : 8,
                   reserved_1 : 8, // Reserved bits
                   start_sin90 : 8,
                   reserved_2 : 8; // Reserved bits
      };
    };
  };

  // MSCNT : R
  struct MSCNT_t {
    constexpr static uint8_t address = 0x6A;
    union {
      uint32_t sr;
      struct {
          uint32_t mscnt     : 10,
                   reserved  : 22; // Reserved bits
      };
    };
  };

  // MSCURACT : R
  struct MSCURACT_t {
    constexpr static uint8_t address = 0x6B;
    union {
      uint32_t sr;
      struct {
          uint32_t cur_a     : 9,
                   reserved_1 : 7, // Reserved bits
                   cur_b     : 9,
                   reserved_2 : 7; // Reserved bits
      };
    };
  };

  struct CHOPCONF_t {
    constexpr static uint8_t address = 0x6C;
    union {
      uint32_t sr : 32;
      struct {
          uint8_t  toff       : 4,
                   hstrt      : 3,
                   hend       : 4,
                   fd3        : 1;
          bool     disfdcc    : 1,
                   reserved_1 : 1, // Reserved bit
                   chm        : 1;
          uint8_t  tbl        : 2,
                   reserved_2 : 1; // Reserved bit
          bool     vhighfs    : 1,
                   vhighchm   : 1;
          uint8_t  tpfd       : 4,
                   mres       : 4,
                   intpol     : 1,
                   dedge      : 1;
          bool     diss2g     : 1,
                   diss2vs    : 1;
      };
    };
  };

  struct COOLCONF_t {
    constexpr static uint8_t address = 0x6D;
    union {
      uint16_t sr : 10;
      struct {
          uint32_t semin      : 4,
                   reserved_1 : 1, // Reserved bit
                   seup       : 2,
                   reserved_2 : 1, // Reserved bit
                   semax      : 4,
                   reserved_3 : 1, // Reserved bit
                   sedn       : 2,
                   seimin     : 1,
                   sgt        : 7,
                   reserved_4 : 1, // Reserved bit
                   sfilt      : 1,
                   reserved_5 : 7; // Reserved bits
      };
    };
  };

  // DCCTRL : W
  struct DCCTRL_t {
    constexpr static uint8_t address = 0x6E;
    union {
      uint32_t sr;
      struct {
          uint32_t dc_time   : 9,
                   reserved_1 : 7, // Reserved bits
                   dc_sg     : 8,
                   reserved_2 : 8; // Reserved bits
      };
    };
  };

  // DRV_STATUS : R
  struct DRV_STATUS_t {
    constexpr static uint8_t address = 0x6F;
    union {
      uint32_t sr;
      struct {
          uint32_t sg_result  : 10,
                   reserved_1 : 2, // Reserved bits
                   s2vsa      : 1,
                   s2vsb      : 1,
                   stealth    : 1,
                   fsactive   : 1,
                   cs_actual  : 5,
                   reserved_2 : 3, // Reserved bits
                   stallguard : 1,
                   ot         : 1,
                   otpw       : 1,
                   s2ga       : 1,
                   s2gb       : 1,
                   ola        : 1,
                   olb        : 1,
                   stst       : 1;
      };
    };
  };

  struct PWMCONF_t {
    constexpr static uint8_t address = 0x70;
    union {
      uint32_t sr : 22;
      struct {
          uint32_t pwm_ofs     : 8,
                   pwm_grad    : 8,
                   pwm_freq    : 2;
          bool pwm_autoscale   : 1,
               pwm_autograd    : 1;
          uint32_t freewheel   : 2,
                   reserved_1  : 2, // Reserved bits
                   pwm_reg     : 4,
                   pwm_lim     : 4;
      };
    };
  };

  struct PWM_SCALE_t {
    constexpr static uint8_t address = 0x71;
    union {
      uint32_t sr : 22;
      struct {
          uint8_t pwm_scale_sum   : 8,
                   reserved_1     : 8, // Reserved bits
                   pwm_scale_auto  : 9,
                   reserved_2     : 7; // Reserved bits
      };
    };
  };

  struct PWM_AUTO_t {
    constexpr static uint8_t address = 0x72;
    union {
      uint32_t sr : 24;
      struct {
          uint8_t pwm_ofs_auto  : 8,
                   reserved_1   : 8, // Reserved bits
                   pwm_grad_auto : 8,
                   reserved_2   : 8; // Reserved bits
      };
    };
  };

  struct LOST_STEPS_t {
    constexpr static uint8_t address = 0x73;
    union {
      uint32_t sr : 24;
      struct {
          uint32_t lost_steps : 20,
                   reserved   : 12; // Reserved bits
      };
    };
  };

  struct DRIVER_STATUS_t {
    union {
      uint8_t sr : 24;
      struct {
          uint8_t reset_flag       : 1,
                  driver_error     : 1,
                  sg2              : 1,
                  standstill       : 1,
                  velocity_reached : 1,
                  position_reached : 1,
                  status_stop_l    : 1,
                  status_stop_r    : 1;
      };
    };
  };

  struct CONFIG_t {
      uint32_t f_clk;
      uint16_t microsteps;
      uint16_t r_sense;           // mOhm
      uint16_t current;           // mA
      uint8_t hold_current_pct;   // percent
      //trinamic_mode_t mode;
      //trinamic_motor_t motor;
      //const trinamic_cfg_params_t *cfg_params;
  };

  struct TMC5160_t {
      // driver registers
      GCONF_t           gconf;
      GSTAT_t           gstat;
      IOIN_t            ioin;
      GLOBAL_SCALER_t   global_scaler;
      IHOLD_IRUN_t      ihold_irun;
      TPOWERDOWN_t      tpowerdown;
      TSTEP_t           tstep;
      TPWMTHRS_t        tpwmthrs;
      TCOOLTHRS_t       tcoolthrs;
      THIGH_t           thigh;
      VDCMIN_t          vdcmin;

      void  *cs_pin;
      /* 
       xdirect;
       mslut[8];
       mslutsel;
       mslutstart;
       encm_ctrl;
       */
      MSCNT_t           mscnt;
      MSCURACT_t        mscuract;
      DCCTRL_t          dcctrl;
      DRV_STATUS_t      drv_status;
      CHOPCONF_t        chopconf;
      COOLCONF_t        coolconf;
      PWMCONF_t         pwmconf;
      PWMCONF_t         pwm_scale;
      LOST_STEPS_t      lost_steps;
      DRIVER_STATUS_t   driver_status;
      CONFIG_t config;
  };
  typedef union {
      uint8_t value;
      struct {
          uint8_t
          idx   :7,
          write :1;
      };
  } TMC_addr_t;

  typedef union {
      uint32_t value;
      uint8_t data[4];
  } TMC_payload_t;


  typedef struct {
    TMC_addr_t addr;
    TMC_payload_t payload;
  } TMC_spi_datagram_t;
  
  typedef union {
    uint8_t value;
    struct {
        uint8_t
        idx   :7,  // Register index/address
        write :1;  // Write flag (1 for write, 0 for read)
    };
  } tmc5160_regaddr_t;


} //namespace
//////////////////////////////////////////////////////
