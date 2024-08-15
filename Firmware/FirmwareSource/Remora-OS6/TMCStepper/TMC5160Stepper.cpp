#include "TMCStepper.h"
#include "TMC_MACROS.h"


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

static const TMC5160_n::TMC5160_t tmc5160_defaults = {
    .config.f_clk = TMC5160_F_CLK,
    .config.r_sense = TMC5160_R_SENSE,
    .config.current = TMC5160_CURRENT,
    .config.hold_current_pct = TMC5160_HOLD_CURRENT_PCT,
    .config.microsteps = TMC5160_MICROSTEPS,

    // register adresses
    .gconf.en_pwm_mode = TMC5160_EN_PWM_MODE,

    .ihold_irun.iholddelay = TMC5160_IHOLDDELAY,
    .tpwmthrs.tpwmthrs = TMC5160_TPWM_THRS,
    .tcoolthrs.tcoolthrs = TMC5160_COOLSTEP_THRS,
    .chopconf.intpol = TMC5160_INTPOL,
    .chopconf.toff = TMC5160_TOFF,
    .chopconf.chm = TMC5160_CHM,
    .chopconf.tbl = TMC5160_TBL,
    .chopconf.hend = TMC5160_HEND + 3,
    .chopconf.vhighfs = TMC5160_VHIGHFS,
    .chopconf.diss2g = TMC5160_DISS2G,
    .chopconf.diss2vs = TMC5160_DISS2VS,
#if TMC5160_CHM == 0
    .chopconf.hstrt = TMC5160_HSTRT - 1,
    .chopconf.fd3   = TMC5160_FD3,
    .chopconf.tpfd  = TMC5160_TPFD,
#else
    .chopconf.fd3 = (TMC5160_TFD & 0x08) >> 3,
    .chopconf.hstrt = TMC5160_TFD & 0x07,
#endif
    .gconf.multistep_filt = TMC5160_MULTISTEP,
    .coolconf.semin = TMC5160_SEMIN,
    .coolconf.seup = TMC5160_SEUP,
    .coolconf.semax = TMC5160_SEMAX,
    .coolconf.sedn = TMC5160_SEDN,
    .coolconf.seimin = TMC5160_SEIMIN,
    .coolconf.sfilt = TMC5160_SFILT,
    .pwmconf.pwm_autoscale = TMC5160_PWM_AUTOSCALE,
    .pwmconf.pwm_lim = TMC5160_PWM_LIM,
    .pwmconf.pwm_reg = TMC5160_PWM_REG,
    .pwmconf.pwm_autograd = TMC5160_PWM_AUTOGRAD,
    .pwmconf.pwm_freq = TMC5160_PWM_FREQ,
    .pwmconf.pwm_grad = TMC5160_PWM_GRAD,
    .pwmconf.pwm_ofs = TMC5160_PWM_OFS,
    .pwmconf.freewheel = TMC5160_FREEWHEEL,
};


// Protected
// addr needed for TMC5160
TMC5160Stepper::TMC5160Stepper(string cs_pin, string spi_bus, uint8_t r_sense, uint8_t current, uint16_t microsteps, bool stealthchop) : 
   TMCStepper(r_sense)
    {

      this->driver = new TMC5160_n::TMC5160_t();
      this->spi = new TMCSPI(spi_bus,cs_pin);

      this->driver->config.current = current;
      this->driver->config.r_sense = r_sense;
      if (MicrostepsIsValid(microsteps))
          SetMicrosteps(microsteps);
      else
          SetMicrosteps(microsteps);

      this->setStealthChop(stealthchop, TMC5160_PWM_FREQ, TMC5160_PWM_AUTOSCALE, TMC5160_PWM_AUTOGRAD);
		 
		  //driver_map_t this->driver, uint16_t current, uint8_t microsteps, uint8_t r_sense)
			defaults();
    }

void TMC5160Stepper::_set_rms_current ()
{
    const uint32_t V_fs = 325; // 0.325 * 1000
    uint_fast8_t CS = 31;
    uint32_t scaler = 0; // = 256

    uint16_t RS_scaled = ((float)this->driver->config.r_sense / 1000.f) * 0xFFFF; // Scale to 16b
    uint32_t numerator = 11585; // 32 * 256 * sqrt(2)
    numerator *= RS_scaled;
    numerator >>= 8;
    numerator *= this->driver->config.current;

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

    this->driver->global_scaler.scaler = scaler;
    this->driver->ihold_irun.irun = CS > 31 ? 31 : CS;
    this->driver->ihold_irun.ihold = (driver->ihold_irun.irun * this->driver->config.hold_current_pct) / 100;
}

void TMC5160Stepper::defaults ()
{
    memcpy(this->driver, &tmc5160_defaults, sizeof(TMC5160_n::TMC5160_t));

    this->_set_rms_current();

}

bool TMC5160Stepper::isEnabled() { return !this->enn() && this->toff(); }

bool TMC5160Stepper::enn() {
    return this->driver->ioin.drv_enn;
}

bool TMC5160Stepper::dir() {
    return this->driver->ioin.refl_dir;
}

bool TMC5160Stepper::toff() {
    return this->driver->chopconf.toff;
}

bool TMC5160Stepper::step() {
    return this->driver->ioin.refl_step;
}

uint8_t TMC5160Stepper::version() {
    return this->driver->ioin.version;
}

// CoolStep configuration
void TMC5160Stepper::setCoolStep(uint8_t semin, uint8_t semax, uint8_t seup, uint8_t sedn, bool seimin) {
    driver->coolconf.semin = semin;
    driver->coolconf.semax = semax;
    driver->coolconf.seup = seup;
    driver->coolconf.sedn = sedn;
    driver->coolconf.seimin = seimin;
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->coolconf);
}

// Stall detection configuration
void TMC5160Stepper::setStallGuard(uint8_t sg_threshold, uint32_t sg_filter_time) {
    driver->coolconf.sgt = sg_threshold;
    driver->tcoolthrs.tcoolthrs = sg_filter_time;
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->coolconf);
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->tcoolthrs);
}

bool TMC5160Stepper::isStallGuardActive() {
    this->ReadRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->drv_status);
    return driver->drv_status.stallguard;
}

uint32_t TMC5160Stepper::getStallGuardResult() {
    this->ReadRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->drv_status);
    return driver->drv_status.sg_result;
}

// Velocity-dependent settings
void TMC5160Stepper::setThresholdSpeed(uint32_t tpwmthrs, uint32_t tcoolthrs, uint32_t thigh) {
    driver->tpwmthrs.tpwmthrs = tpwmthrs;
    driver->tcoolthrs.tcoolthrs = tcoolthrs;
    driver->thigh.thigh = thigh;
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->tpwmthrs);
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->tcoolthrs);
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->thigh);
}

// Power stage settings
void TMC5160Stepper::setPowerStageParameters(uint8_t tbl, uint8_t toff, uint8_t hstrt, uint8_t hend) {
    driver->chopconf.tbl = tbl;
    driver->chopconf.toff = toff;
    driver->chopconf.hstrt = hstrt;
    driver->chopconf.hend = hend;
    WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->chopconf);
}

// StealthChop configuration
void TMC5160Stepper::setStealthChop(bool enable, uint8_t freq, uint8_t autoscale, uint8_t autograd) {
    driver->gconf.en_pwm_mode = enable;
    driver->pwmconf.pwm_freq = freq;
    driver->pwmconf.pwm_autoscale = autoscale;
    driver->pwmconf.pwm_autograd = autograd;
    WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->gconf);
    WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->pwmconf);
}

// Driver status
TMC5160_n::DRV_STATUS_t TMC5160Stepper::getDriverStatus() {
    this->ReadRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->drv_status);
    return driver->drv_status;
}

// TSTEP measurement (for speed calculation)
uint32_t TMC5160Stepper::getTStep() {
    this->ReadRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->tstep);
    return driver->tstep.tstep;
}

// Calculate current speed (in steps/second)
float TMC5160Stepper::getCurrentSpeed() {
    uint32_t tstep = getTStep();
    if (tstep == 0) return 0.0f; // Avoid division by zero
    return 12000000.0f / (256.0f * tstep); // Assuming internal clock of 12MHz
}

bool TMC5160Stepper::Init ()
{
    // Read drv_status to check if this->driver is online
    this->ReadRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->drv_status);
    if(driver->drv_status.sr == 0 || this->driver->drv_status.sr == 0xFFFFFFFF)
        return false;

    // Perform a status register read to clear reset flag
    this->ReadRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->gstat);

    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->gconf);
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->chopconf);
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->coolconf);
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->pwmconf);
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->tpowerdown);
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->tpwmthrs);

    this->SetCurrent(this->driver->config.current, this->driver->config.hold_current_pct);

    // Read back chopconf to check if this->driver is online
    uint32_t chopconf = this->driver->chopconf.sr;
    this->ReadRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->chopconf);

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

    WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->global_scaler);
    WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->ihold_irun);
}

float TMC5160Stepper::GetTPWMTHRS (float steps_mm)
{
    return this->tmc_calc_tstep_inv(this->driver->tpwmthrs.tpwmthrs, steps_mm);
}

void TMC5160Stepper::SetTPWMTHRS (float mm_sec, float steps_mm) // -> pwm threshold
{
    this->driver->tpwmthrs.tpwmthrs = this->tmc_calc_tstep(mm_sec, steps_mm);
    WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&this->driver->tpwmthrs);
}

void TMC5160Stepper::SetTHIGH (float mm_sec, float steps_mm) // -> pwm threshold
{
    this->driver->thigh.thigh = this->tmc_calc_tstep(mm_sec, steps_mm);
    WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&this->driver->thigh);
}

void TMC5160Stepper::SetTCOOLTHRS (float mm_sec, float steps_mm) // -> pwm threshold
{
    this->driver->tcoolthrs.tcoolthrs = this->tmc_calc_tstep(mm_sec, steps_mm);
    WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&this->driver->tcoolthrs);
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
    WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&this->driver->chopconf);
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

    WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&this->driver->chopconf);
}



void TMC5160Stepper::WriteRegister (TMC5160_n::TMC_spi_datagram_t *reg)
{
    tmcSPIWrite((TMC5160_n::TMC_spi_datagram_t *)reg);
}

TMC5160_n::TMC_payload_t  TMC5160Stepper::ReadRegister (TMC5160_n::TMC_spi_datagram_t *reg)
{
    TMC5160_n::TMC_payload_t status;

    status.value = tmcSPIRead((TMC5160_n::TMC_spi_datagram_t *)reg);

    return status;
}

// Returns pointer to shadow register or NULL if not found
/*TMC5160_n::TMC_spi_datagram_t * TMC5160Stepper::GetRegPtr (TMC5160_n::tmc5160_regaddr_t reg)
{
     TMC5160_n::TMC_spi_datagram_t *ptr = (TMC5160_n::TMC_spi_datagram_t *)this->driver;

    while(ptr && ptr->addr != reg) {
        ptr++;
        if(ptr->addr == TMC5160_n::LOST_STEPS_t && ptr->addr != reg)
            ptr = NULL;
    }

    return ptr;
}

**/
void TMC5160Stepper::tmcSPIWrite (TMC5160_n::TMC_spi_datagram_t *datagram)
{
    this->spi->spiWrite(datagram->addr.value, datagram->payload.value);
}

uint32_t TMC5160Stepper::tmcSPIRead (TMC5160_n::TMC_spi_datagram_t *datagram)
{
    return this->spi->spiRead(datagram->addr.value);
}

//void TMC2208Stepper::write(uint8_t addr, uint32_t regVal) {
void TMC5160Stepper::write(uint8_t addr, uint32_t value) {
    TMC5160_n::TMC_addr_t reg;
    TMC5160_n::TMC_payload_t payload;
    TMC5160_n::TMC_spi_datagram_t* spi_data = new TMC5160_n::TMC_spi_datagram_t();
    reg.value = addr;
    payload.value = value;
    spi_data->addr = reg;
    spi_data->payload = payload;
    tmcSPIWrite(spi_data);
}
uint32_t TMC5160Stepper::read(uint8_t addr) {

    TMC5160_n::TMC_addr_t reg;
    TMC5160_n::TMC_spi_datagram_t* spi_data = new TMC5160_n::TMC_spi_datagram_t();
    reg.value = addr;
    spi_data->addr = reg;
    return this->tmcSPIRead(spi_data);
}
void TMC5160Stepper::vsense(bool fake) { 
   // This doesn't match TMC5160 drivers;
   fake = false;
}
bool TMC5160Stepper::vsense(void) {
    return false;
}
uint32_t TMC5160Stepper::DRV_STATUS() {
    TMC5160_n::TMC_payload_t payload =  this->ReadRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->drv_status);
    return payload.value;
}
void TMC5160Stepper::hend(uint8_t val) {
    this->driver->chopconf.hstrt = val;
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->chopconf);
}
uint8_t TMC5160Stepper::hend() {
    return this->driver->chopconf.hend;
}
void TMC5160Stepper::hstrt(uint8_t val) {

    this->driver->chopconf.hstrt = val;
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->chopconf);
}
uint8_t TMC5160Stepper::hstrt() {
    return this->driver->chopconf.hstrt;
}
void TMC5160Stepper::mres(uint8_t val) {

    this->driver->chopconf.mres = val;
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->chopconf);
}
uint8_t TMC5160Stepper::mres() {
    return this->driver->chopconf.mres;

}
void TMC5160Stepper::tbl(uint8_t val) {

    this->driver->chopconf.tbl = val;
    this->WriteRegister((TMC5160_n::TMC_spi_datagram_t *)&driver->chopconf);
}
uint8_t TMC5160Stepper::tbl() {
    return this->driver->chopconf.tbl;
}

