#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <cstdint>
#include <string>
//#include <iostream>

#include "modules/module.h"
#include "sensors/tempSensor.h"
#include "sensors/thermistor/thermistor.h"

#include "remora.h"

unique_ptr<Module> createTemperature(const JsonObject& config);

class Temperature : public Module
{
  private:

    const char* sensorType;       // temperature sensor type
    const char* pinSensor;	             // physical pins connections

    volatile float* ptrFeedback;       	   // pointer where to put the feedback

    float temperaturePV;

    // thermistor parameters
    float beta;
    float r0;
		float t0;

  public:

    Temperature(volatile float&, int32_t, int32_t, const char*, const char*, float, int, int);  // Thermistor type constructor

    TempSensor* Sensor;

    virtual void update(void);           // Module default interface
    virtual void slowUpdate(void);
};


#endif
