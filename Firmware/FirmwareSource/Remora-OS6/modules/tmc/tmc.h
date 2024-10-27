#ifndef TMCMODULE_H
#define TMCMODULE_H

#include "mbed.h"
#include <cstdint>
#include <string>

#include "module.h"
#include "TMCStepper/TMCStepper.h"

#include "remora.h"


unique_ptr<Module> createTMC2208(const JsonObject& config);
unique_ptr<Module> createTMC2209(const JsonObject& config);
unique_ptr<Module> createTMC5160(const JsonObject& config);

class TMC : public Module
{
  protected:

    float       Rsense;

  public:

    virtual void update(void) = 0;           // Module default interface
    virtual void configure(void) = 0;
};


class TMC2208 : public TMC
{
  protected:

    const char* rxtxPin;     // default to half duplex
    uint16_t    mA;
    uint16_t    microsteps;
    bool        stealth;

    TMC2208Stepper* driver;

  public:

    // SW Serial pin, Rsense, mA, microsteps, stealh
    TMC2208(const char*, float, uint16_t, uint16_t, bool);
    ~TMC2208();

    void update(void);           // Module default interface
    void configure(void);
};


class TMC2209 : public TMC
{
  protected:

    const char* rxtxPin;     // default to half duplex
    uint16_t    mA;
    uint16_t    microsteps;
    bool        stealth;
    uint8_t     addr;
    uint16_t    stall;

    TMC2209Stepper* driver;

  public:

    // SW Serial pin, Rsense, addr, mA, microsteps, stealh, hybrid, stall
    // TMC2209(const char*, float, uint8_t, uint16_t, uint16_t, bool, uint16_t);
    TMC2209(const char*, float, uint8_t, uint16_t, uint16_t, bool, uint16_t);
    ~TMC2209();

    void update(void);           // Module default interface
    void configure(void);
};

class TMC5160 : public TMC
{
  protected:

    const char* csPin; 
    const char* spiBus; 
    uint16_t    mA;
    uint16_t    microsteps;
    bool        stealth;
    uint8_t     addr;
    uint16_t    stall;

    TMC5160Stepper* driver;

  public:

    // SW Serial pin, Rsense, addr, mA, microsteps, stealh, hybrid, stall
    // TMC2209(const char*, float, uint8_t, uint16_t, uint16_t, bool, uint16_t);
    TMC5160(const char* csPin, const char* spiBus, float Rsense, uint16_t mA, uint16_t microsteps, bool stealth);
    ~TMC5160();

    void update(void);           // Module default interface
    void configure(void);
};

#endif
