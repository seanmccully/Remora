#include "TMCStepper.h"

using namespace TMC5160_n;
typedef enum {
    TMC5160_Microsteps_1 = 1,
    TMC5160_Microsteps_2 = 2,
    TMC5160_Microsteps_4 = 4,
    TMC5160_Microsteps_8 = 8,
    TMC5160_Microsteps_16 = 16,
    TMC5160_Microsteps_32 = 32,
    TMC5160_Microsteps_64 = 64,
    TMC5160_Microsteps_128 = 128,
    TMC5160_Microsteps_256 = 256
} tmc5160_microsteps_t;

// General
#define TMC5160_F_CLK               12000000UL              // factory tuned to 12MHz - see datasheet for calibration procedure if required
#define TMC5160_MODE                0     // 0 = TMCMode_StealthChop, 1 = TMCMode_CoolStep, 3 = TMCMode_StallGuard
#define TMC5160_MICROSTEPS          TMC5160_Microsteps_32
#define TMC5160_R_SENSE             50                      // mOhm
#define TMC5160_CURRENT             2500                     // mA RMS
#define TMC5160_HOLD_CURRENT_PCT    50
#define TMC5160_MULTISTEP           1

// CHOPCONF
#define TMC5160_INTPOL              1   // Step interpolation: 0 = off, 1 = on
#define TMC5160_TOFF                3   // Off time: 1 - 15, 0 = MOSFET disable (8)
#define TMC5160_TBL                 2   // Blanking time: 0 = 16, 1 = 24, 2 = 36, 3 = 54 clocks
#define TMC5160_CHM                 0   // Chopper mode: 0 = spreadCycle, 1 = constant off time
// TMC5160_CHM 0 defaults
#define TMC5160_HSTRT               5   // Hysteresis start: 1 - 8
#define TMC5160_HEND                2   // Hysteresis end: -3 - 12
#define TMC5160_HMAX               16   // HSTRT + HEND
// TMC5160_CHM 1 defaults
#define TMC5160_FD3                 0  // fd3 & hstrt: 0 - 15
#define TMC5160_TPFD                4  //
#define TMC5160_TFD                13  // fd3 & hstrt: 0 - 15

// IHOLD_IRUN
#define TMC5160_IHOLDDELAY          6

// TPWMTHRS
#define TMC5160_TPWM_THRS           0   // tpwmthrs: 0 - 2^20 - 1 (20 bits)
#define TMC5160_VHIGHFS             0   //
#define TMC5160_VHIGHCHM            0   //
#define TMC5160_DISS2G              0   //
#define TMC5160_DISS2VS             0   //
#define TMC5160_FREEWHEEL           0   //
#define TMC5160_SGT                 0   //
                                        //
#define TMC_THRESHOLD_MIN           0
// PWMCONF - StealthChop defaults
#define TMC5160_STEALTH_THRS       TMC_THRESHOLD_MIN   // tpwmthrs: 0 - 2^20 - 1 (20 bits)
#define TMC5160_PWM_FREQ            0   // 0 = 1/1024, 1 = 2/683, 2 = 2/512, 3 = 2/410 fCLK
#define TMC5160_PWM_AUTOGRAD        1   // boolean (0 or 1)
#define TMC5160_PWM_GRAD            0  // 0 - 255
#define TMC5160_PWM_LIM             12  // 0 - 15
#define TMC5160_PWM_REG             4   // 1 - 15
#define TMC5160_PWM_OFS             30  // 0 - 255

// TCOOLTHRS
#define TMC5160_COOLSTEP_THRS       TMC_THRESHOLD_MIN   // tpwmthrs: 0 - 2^20 - 1 (20 bits)


// COOLCONF - CoolStep defaults
#define TMC5160_SEMIN               0   // 0 = coolStep off, 1 - 15 = coolStep on
#define TMC5160_SEUP                0   // 0 - 3 (1 - 8)
#define TMC5160_SEMAX               0   // 0 - 15
#define TMC5160_SEDN                0   // 0 - 3
#define TMC5160_SEIMIN              0   // boolean (0 or 1)
#define TMC5160_SFILT               0   // boolean (0 or 1)

// end of default values

#if TMC5160_MODE == 0   // StealthChop
#define TMC5160_PWM_AUTOSCALE 1
#define TMC5160_EN_PWM_MODE   1
#elif TMC5160_MODE == 1 // CoolStep
#define TMC5160_PWM_AUTOSCALE 0
#define TMC5160_EN_PWM_MODE   0
#else                   //StallGuard
#define TMC5160_PWM_AUTOSCALE 0
#define TMC5160_EN_PWM_MODE   0
#endif

// Initialize with constructor instead of static initialization
TMC5160_t* TMC5160Stepper::createTMC5160Defaults() {
    TMC5160_t* defaults = new TMC5160_t();

    // config
    defaults->config.f_clk = TMC5160_F_CLK;
    defaults->config.r_sense = TMC5160_R_SENSE;
    defaults->config.current = TMC5160_CURRENT;
    defaults->config.hold_current_pct = TMC5160_HOLD_CURRENT_PCT;
    defaults->config.microsteps = TMC5160_MICROSTEPS;

    // gconf
    defaults->gconf.en_pwm_mode = TMC5160_EN_PWM_MODE;
    defaults->gconf.multistep_filt = TMC5160_MULTISTEP;

    // ihold_irun
    defaults->ihold_irun.iholddelay = TMC5160_IHOLDDELAY;

    // tpwmthrs
    defaults->tpwmthrs.tpwmthrs = TMC5160_TPWM_THRS;

    // tcoolthrs
    defaults->tcoolthrs.tcoolthrs = TMC5160_COOLSTEP_THRS;

    // chopconf
    defaults->chopconf.intpol = TMC5160_INTPOL;
    defaults->chopconf.toff = TMC5160_TOFF;
    defaults->chopconf.chm = TMC5160_CHM;
    defaults->chopconf.tbl = TMC5160_TBL;
    defaults->chopconf.hend = TMC5160_HEND + 3;
    defaults->chopconf.vhighfs = TMC5160_VHIGHFS;
    defaults->chopconf.diss2g = TMC5160_DISS2G;
    defaults->chopconf.diss2vs = TMC5160_DISS2VS;

    #if TMC5160_CHM == 0
        defaults->chopconf.hstrt = TMC5160_HSTRT - 1;
        defaults->chopconf.fd3 = TMC5160_FD3;
        defaults->chopconf.tpfd = TMC5160_TPFD;
    #else
        defaults->chopconf.fd3 = (TMC5160_TFD & 0x08) >> 3;
        defaults->chopconf.hstrt = TMC5160_TFD & 0x07;
    #endif

    // coolconf
    defaults->coolconf.semin = TMC5160_SEMIN;
    defaults->coolconf.seup = TMC5160_SEUP;
    defaults->coolconf.semax = TMC5160_SEMAX;
    defaults->coolconf.sedn = TMC5160_SEDN;
    defaults->coolconf.seimin = TMC5160_SEIMIN;
    defaults->coolconf.sfilt = TMC5160_SFILT;

    // pwmconf
    defaults->pwmconf.pwm_autoscale = TMC5160_PWM_AUTOSCALE;
    defaults->pwmconf.pwm_lim = TMC5160_PWM_LIM;
    defaults->pwmconf.pwm_reg = TMC5160_PWM_REG;
    defaults->pwmconf.pwm_autograd = TMC5160_PWM_AUTOGRAD;
    defaults->pwmconf.pwm_freq = TMC5160_PWM_FREQ;
    defaults->pwmconf.pwm_grad = TMC5160_PWM_GRAD;
    defaults->pwmconf.pwm_ofs = TMC5160_PWM_OFS;
    defaults->pwmconf.freewheel = TMC5160_FREEWHEEL;

    return defaults;
}



// Protected
// addr needed for TMC5160
TMC5160Stepper::TMC5160Stepper(const char* cs_pin, const char* spi_bus, float r_sense, uint16_t current, uint8_t microsteps, bool stealthchop) :
   Rsense(r_sense)
    {
            this->driver = new TMC5160_t();
            this->spi = new TMCSPI(spi_bus, cs_pin);


            this->driver->config.current = current;
            this->driver->config.r_sense = r_sense;
            if (MicrostepsIsValid(microsteps)) {
                SetMicrosteps(microsteps);
            } else {
                SetMicrosteps(16);
            }

            defaults();
            this->setStealthChop(stealthchop, TMC5160_PWM_FREQ, TMC5160_PWM_AUTOSCALE, TMC5160_PWM_AUTOGRAD);

    }


void TMC5160Stepper::cleanup() {
    if (this->driver) {
        delete this->driver;
        this->driver = nullptr;
    }
    if (this->spi) {
        delete this->spi;
        this->spi = nullptr;
    }
}


void TMC5160Stepper::_set_rms_current ()
{
    const uint32_t V_fs = 325; // 0.325 * 1000
    uint_fast8_t CS = 31;
    uint32_t scaler = 0; // = 256

    uint16_t RS_scaled = ((float)driver->config.r_sense / 1000.f) * 0xFFFF; // Scale to 16b
    uint32_t numerator = 11585; // 32 * 256 * sqrt(2)
    numerator *= RS_scaled;
    numerator >>= 8;
    numerator *= driver->config.current;

    int max_iterations = 100; // Maximum number of iterations to prevent infinite loop
    do {
        uint32_t denominator = V_fs * 0xFFFF >> 8;
        denominator *= CS + 1;
        scaler = numerator / denominator;
        if (scaler > 255)
            scaler = 0; // Maximum
        else if (scaler < 128)
            CS--;  // Try again with smaller CS
        max_iterations--;
    } while(scaler && scaler < 128 && max_iterations > 0);

    driver->global_scaler.scaler = scaler;
    driver->ihold_irun.irun = CS > 31 ? 31 : CS;
    driver->ihold_irun.ihold = (driver->ihold_irun.irun * driver->config.hold_current_pct) / 100;
}

void TMC5160Stepper::defaults ()
{
    this->driver = createTMC5160Defaults();

    this->_set_rms_current();

    Init();
}

uint8_t TMC5160Stepper::test_connection() {

  switch (DRV_STATUS()) {
      case 0xFFFFFFFF: return 1;
      case 0: return 2;
      default: return 0;
  }
}



// CoolStep configuration
void TMC5160Stepper::setCoolStep(uint8_t semin, uint8_t semax, uint8_t seup, uint8_t sedn, bool seimin) {
    driver->coolconf.semin = semin;
    driver->coolconf.semax = semax;
    driver->coolconf.seup = seup;
    driver->coolconf.sedn = sedn;
    driver->coolconf.seimin = seimin;

    if (!WriteRegisterValidate((TMC_spi_datagram_t *)&driver->coolconf))
    		MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"TMC5160 Driver Error",0x1243);
}

// Stall detection configuration
void TMC5160Stepper::setStallGuard(uint8_t sg_threshold, uint32_t sg_filter_time) {
    driver->coolconf.sgt = sg_threshold;
    driver->tcoolthrs.tcoolthrs = sg_filter_time;

    if (!WriteRegisterValidate((TMC_spi_datagram_t *)&driver->coolconf))
    		MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"TMC5160 Driver Error",0x1243);


   if (!WriteRegisterValidate((TMC_spi_datagram_t *)&driver->tcoolthrs))
    		MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"TMC5160 Driver Error",0x1243);
}

bool TMC5160Stepper::isStallGuardActive() {
    return stallguard();
}

uint32_t TMC5160Stepper::getStallGuardResult() {
		return sg_result();
}

// Velocity-dependent settings
void TMC5160Stepper::setThresholdSpeed(uint32_t tpwmthrs, uint32_t tcoolthrs, uint32_t thigh) {
    driver->tpwmthrs.tpwmthrs = tpwmthrs;
    driver->tcoolthrs.tcoolthrs = tcoolthrs;
    driver->thigh.thigh = thigh;

    if (!WriteRegisterValidate((TMC_spi_datagram_t *)&driver->tpwmthrs))
    		MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"TMC5160 Driver Error [TPWMTHRS]",0x1243);

    if (!WriteRegisterValidate((TMC_spi_datagram_t *)&driver->tcoolthrs))
    		MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"TMC5160 Driver Error [TCOOLTHRS]",0x1243);

    if (!WriteRegisterValidate((TMC_spi_datagram_t *)&driver->thigh))
    		MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"TMC5160 Driver Error [THIGH]",0x1243);

}

// Power stage settings
void TMC5160Stepper::setPowerStageParameters(uint8_t tbl, uint8_t toff, uint8_t hstrt, uint8_t hend) {
    driver->chopconf.tbl = tbl;
    driver->chopconf.toff = toff;
    driver->chopconf.hstrt = hstrt;
    driver->chopconf.hend = hend;
    if (!WriteRegisterValidate((TMC_spi_datagram_t *)&driver->chopconf))
    		MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"TMC5160 Driver Error [CHOPCONF]",0x1243);
}

void TMC5160Stepper::setStealthChop(bool enable, uint8_t freq, uint8_t autoscale, uint8_t autograd) {
    this->driver->gconf.en_pwm_mode = enable;
    this->driver->pwmconf.pwm_freq = freq;
    this->driver->pwmconf.pwm_autoscale = autoscale;
    this->driver->pwmconf.pwm_autograd = autograd;

    if (!WriteRegisterValidate((TMC_spi_datagram_t *)&this->driver->gconf))
    		MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"TMC5160 Driver Error [GCONF]",0x1243);

    if (!WriteRegisterValidate((TMC_spi_datagram_t *)&this->driver->pwmconf))
    		MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"TMC5160 Driver Error [PWMCONF]",0x1243);
}


// TSTEP measurement (for speed calculation)
uint32_t TMC5160Stepper::getTStep() {
    this->ReadRegister((TMC_spi_datagram_t *)&driver->tstep);
    return driver->tstep.tstep;
}

// Calculate current speed (in steps/second)
float TMC5160Stepper::getCurrentSpeed() {
    uint32_t tstep = getTStep();
    if (tstep == 0) return 0.0f; // Avoid division by zero
    return 12000000.0f / (256.0f * tstep); // Assuming internal clock of 12MHz
}

bool TMC5160Stepper::Init()
{
    /*spi->spiWrite(TMC5160_n::GCONF_t::address, this->driver->gconf.sr);
    spi->spiWrite(TMC5160_n::CHOPCONF_t::address, this->driver->chopconf.sr);
    spi->spiWrite(TMC5160_n::COOLCONF_t::address, this->driver->coolconf.sr);
    spi->spiWrite(TMC5160_n::PWMCONF_t::address, this->driver->pwmconf.sr);
    spi->spiWrite(TMC5160_n::TPOWERDOWN_t::address, this->driver->tpowerdown.sr);
    spi->spiWrite(TMC5160_n::TPWMTHRS_t::address, this->driver->tpwmthrs.sr);
    */

    // Perform a status register read to clear reset flag
    ReadRegister((TMC_spi_datagram_t *)&driver->gstat);

    WriteRegister((TMC_spi_datagram_t *)&driver->gconf);
    WriteRegister((TMC_spi_datagram_t *)&driver->chopconf);
    WriteRegister((TMC_spi_datagram_t *)&driver->coolconf);
    WriteRegister((TMC_spi_datagram_t *)&driver->pwmconf);
    WriteRegister((TMC_spi_datagram_t *)&driver->tpowerdown);
    WriteRegister((TMC_spi_datagram_t *)&driver->tpwmthrs);

    SetCurrent(driver->config.current, driver->config.hold_current_pct);

    // Read back chopconf to check if this->driver is online
    uint32_t chopconf = driver->chopconf.sr;
    ReadRegister((TMC_spi_datagram_t *)&driver->chopconf);

    return this->driver->chopconf.sr == chopconf;

}

uint_fast16_t TMC5160Stepper::cs2rms (uint8_t CS)
{
    uint32_t numerator = (this->driver->global_scaler.scaler ? this->driver->global_scaler.scaler : 256) * (CS + 1);
    numerator *= 325;
    numerator >>= (8 + 5); // Divide by 256 and 32
    numerator *= 1000000;
    uint32_t denominator = this->driver->config.r_sense;
    denominator *= 1414;

    return numerator / denominator;
}

uint16_t TMC5160Stepper::GetCurrent ()
{
    return cs2rms(this->driver->ihold_irun.irun);
}

// r_sense = mOhm, Vsense = mV, current = mA (RMS)
void TMC5160Stepper::SetCurrent (uint16_t mA, uint8_t hold_pct)
{
    this->driver->config.current = mA;
    this->driver->config.hold_current_pct = hold_pct;

    this->_set_rms_current();

    WriteRegister((TMC_spi_datagram_t *)&driver->global_scaler);
    WriteRegister((TMC_spi_datagram_t *)&driver->ihold_irun);
}

float TMC5160Stepper::GetTPWMTHRS (float steps_mm)
{
    return this->tmc_calc_tstep_inv(this->driver->tpwmthrs.tpwmthrs, steps_mm);
}

void TMC5160Stepper::SetTPWMTHRS (float mm_sec, float steps_mm) // -> pwm threshold
{
    this->driver->tpwmthrs.tpwmthrs = this->tmc_calc_tstep(mm_sec, steps_mm);
    WriteRegister((TMC_spi_datagram_t *)&this->driver->tpwmthrs);
}

void TMC5160Stepper::SetTHIGH (float mm_sec, float steps_mm) // -> pwm threshold
{
    this->driver->thigh.thigh = this->tmc_calc_tstep(mm_sec, steps_mm);
    WriteRegister((TMC_spi_datagram_t *)&this->driver->thigh);
}

void TMC5160Stepper::SetTCOOLTHRS (float mm_sec, float steps_mm) // -> pwm threshold
{
    this->driver->tcoolthrs.tcoolthrs = this->tmc_calc_tstep(mm_sec, steps_mm);
    WriteRegister((TMC_spi_datagram_t *)&this->driver->tcoolthrs);
}

uint32_t TMC5160Stepper::tmc_calc_tstep (float mm_sec, float steps_mm)
{
    uint32_t den = (uint32_t)(256.0f * mm_sec * steps_mm);

    return den ? (this->driver->config.microsteps * this->driver->config.f_clk) / den : 0;
}

float TMC5160Stepper::tmc_calc_tstep_inv (uint32_t tstep, float steps_mm)
{
    return tstep == 0 ? 0.0f : (float)(this->driver->config.f_clk * this->driver->config.microsteps) / (256.0f * (float)tstep * steps_mm);
}

bool TMC5160Stepper::MicrostepsIsValid (uint16_t usteps) {
    return this->tmc_microsteps_validate(usteps);
}

uint8_t TMC5160Stepper::tmc_microsteps_to_mres(uint16_t microsteps) {
    switch(microsteps) {
        case 256: return 0;
        case 128: return 1;
        case 64:  return 2;
        case 32:  return 3;
        case 16:  return 4;
        case 8:   return 5;
        case 4:   return 6;
        case 2:   return 7;
        case 1:   return 8;
        default:  return 0; // Default to 256 microsteps
    }
}

// Convert MRES value back to microsteps
uint16_t TMC5160Stepper::tmc_mres_to_microsteps(uint8_t mres) {
    return 256 >> mres;
}

// Validate microstep value
bool TMC5160Stepper::tmc_microsteps_validate(uint16_t microsteps) {
    switch(microsteps) {
        case 1:
        case 2:
        case 4:
        case 8:
        case 16:
        case 32:
        case 64:
        case 128:
        case 256:
            return true;
        default:
            return false;
    }
}

void TMC5160Stepper::SetMicrosteps (uint16_t msteps)
{
    this->driver->chopconf.mres = this->tmc_microsteps_to_mres(msteps);
    this->driver->config.microsteps = (tmc5160_microsteps_t)(1 << (8 - this->driver->chopconf.mres));
// TODO: recalc and set hybrid threshold if enabled?
    WriteRegister((TMC_spi_datagram_t *)&this->driver->chopconf);
}

uint32_t TMC5160Stepper::getMicrostepcounter() {
    uint8_t tx_data[5] = {0x6A, 0, 0, 0, 0}; // MSCNT register
    uint8_t rx_data[5];

    this->spi->ResetCSPin();
    for(int i = 0; i < 5; i++) {
        rx_data[i] = this->spi->spiSendReceive(tx_data[i]);
    }
    this->spi->SetCSPin();

    return (rx_data[1] << 24) | (rx_data[2] << 16) |
           (rx_data[3] << 8) | rx_data[4];
}

void TMC5160Stepper::SetConstantOffTimeChopper (uint8_t constant_off_time,
                                                        uint8_t blank_time, uint8_t fast_decay_time,
																												int8_t sine_wave_offset, bool use_current_comparator)
{
    //calculate the value acc to the clock cycles
    if (blank_time >= 54)
        blank_time = 3;
    else if (blank_time >= 36)
        blank_time = 2;
    else if (blank_time >= 24)
        blank_time = 1;
    else
        blank_time = 0;

    if (fast_decay_time > 15)
        fast_decay_time = 15;

    if(this->driver->chopconf.chm)
        this->driver->chopconf.fd3 = (fast_decay_time & 0x8) >> 3;

    this->driver->chopconf.tbl = blank_time;
    this->driver->chopconf.toff = constant_off_time < 2 ? 2 : (constant_off_time > 15 ? 15 : constant_off_time);
    this->driver->chopconf.hstrt = fast_decay_time & 0x7;
    this->driver->chopconf.hend = (sine_wave_offset < -3 ? -3 : (sine_wave_offset > 12 ? 12 : sine_wave_offset)) + 3;

    WriteRegister((TMC_spi_datagram_t *)&this->driver->chopconf);
}



bool TMC5160Stepper::WriteRegisterValidate(TMC_spi_datagram_t *reg) {
    WriteRegister(reg);
    // Read back and verify
    TMC_payload_t readback = ReadRegister(reg);
    return readback.value == reg->payload.value;
}

void TMC5160Stepper::WriteRegister (TMC_spi_datagram_t *reg)
{
    reg->addr.write = 1;
    spi->spiWrite(reg->addr.value, reg->payload.value);
}

TMC_payload_t  TMC5160Stepper::ReadRegister (TMC_spi_datagram_t *reg)
{
    TMC_payload_t status;

    reg->addr.write = 0;
    status.value = spi->spiRead(reg->addr.value);

    return status;
}


void TMC5160Stepper::vsense(bool fake) {
   // This doesn't match TMC5160 drivers;
   fake = false;
}
bool TMC5160Stepper::vsense(void) {
    return false;
}

bool TMC5160Stepper::stallguard() {
	TMC5160_n::DRV_STATUS_t drv = getDRV_STATUS();
	return drv.stallguard;
}

bool TMC5160Stepper::sg_result() {
	TMC5160_n::DRV_STATUS_t drv = getDRV_STATUS();
	return drv.sg_result;
}

uint32_t TMC5160Stepper::DRV_STATUS() {
    TMC5160_n::DRV_STATUS_t drv = getDRV_STATUS();
    return drv.sr;  // Direct access to the uint32_t representation
    // Or: return uint32_t(ioin);  // Using conversion operator
}

// In TMC5160Stepper.cpp
TMC5160_n::DRV_STATUS_t TMC5160Stepper::getDRV_STATUS() {
    TMC5160_n::DRV_STATUS_t reg;
    reg.sr = spi->spiRead(TMC5160_n::DRV_STATUS_t::address);
    return reg;
}


bool TMC5160Stepper::enn() {
    return getIOIN().drv_enn;
}

bool TMC5160Stepper::dir() {
    return getIOIN().refl_dir;
}

bool TMC5160Stepper::step() {
    return getIOIN().refl_step;
}

uint32_t TMC5160Stepper::IOIN() {
    TMC5160_n::IOIN_t ioin = getIOIN();
    return ioin.sr;  // Direct access to the uint32_t representation
    // Or: return uint32_t(ioin);  // Using conversion operator
}

TMC5160_n::IOIN_t TMC5160Stepper::getIOIN() {
    TMC5160_n::IOIN_t reg;
    reg.sr = spi->spiRead(TMC5160_n::IOIN_t::address);
    return reg;
}

TMC5160_n::CHOPCONF_t TMC5160Stepper::getCHOPCONF() {
    return TMC5160_n::CHOPCONF_t(spi->spiRead(TMC5160_n::CHOPCONF_t::address));
}

uint32_t TMC5160Stepper::CHOPCONF() {
    TMC5160_n::CHOPCONF_t chop = getCHOPCONF();
    return chop.sr;
}
void TMC5160Stepper::CHOPCONF(uint32_t value) {
    TMC5160_n::CHOPCONF_t data = value;
    spi->spiWrite(TMC5160_n::CHOPCONF_t::address, data.sr);
}

void TMC5160Stepper::tbl(uint8_t value) {
    TMC5160_n::CHOPCONF_t data = getCHOPCONF();
    data.tbl = std::min(value, uint8_t(3));
    CHOPCONF(data.sr);
}

uint8_t TMC5160Stepper::tbl() {
    return getCHOPCONF().tbl;
}

// Hysteresis start
void TMC5160Stepper::hstrt(uint8_t value) {
    TMC5160_n::CHOPCONF_t data = getCHOPCONF();
    data.hstrt = std::min(value, uint8_t(7));
    CHOPCONF(data.sr);
}


uint8_t TMC5160Stepper::hstrt() {
    return getCHOPCONF().hstrt;
}

// Hysteresis end
void TMC5160Stepper::hend(uint8_t value) {
    TMC5160_n::CHOPCONF_t data = getCHOPCONF();
    data.hend = std::min(std::max(value + 3, 0), 15);  // Convert from -3..12 to 0..15
    CHOPCONF(data.sr);
}

uint8_t TMC5160Stepper::hend() {
    return getCHOPCONF().hend - 3;  // Convert from 0..15 to -3..12
}

// TOFF setting
void TMC5160Stepper::toff(uint8_t value) {
    TMC5160_n::CHOPCONF_t data = getCHOPCONF();
    data.toff = std::min(value, uint8_t(15));
    CHOPCONF(data.sr);
}

uint8_t TMC5160Stepper::toff() {
    return getCHOPCONF().toff;
}

TMC5160_n::COOLCONF_t TMC5160Stepper::getCOOLCONF() {
    return TMC5160_n::COOLCONF_t(spi->spiRead(TMC5160_n::COOLCONF_t::address));
}



TMC5160Stepper::~TMC5160Stepper() {
		cleanup();
}

