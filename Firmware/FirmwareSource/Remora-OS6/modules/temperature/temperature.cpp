#include "temperature.h"


/***********************************************************************
                MODULE CONFIGURATION AND CREATION FROM JSON
************************************************************************/

unique_ptr<Module> createTemperature(const JsonObject& config) {
    const char* comment = config["Comment"];

    int pv = config["PV[i]"];
    const char* sensor = config["Sensor"];

    if (!strcmp(sensor, "Thermistor"))
    {
        const char* pinSensor = config["Thermistor"]["Pin"];
        float beta =  config["Thermistor"]["beta"];
        int r0 = config["Thermistor"]["r0"];
        int t0 = config["Thermistor"]["t0"];

        // slow module with 1 hz update
        int updateHz = 1;
        return make_unique<Temperature>(&txData->processVariable[pv], PRU_SERVOFREQ, updateHz, sensor, pinSensor, beta, r0, t0);
    }
    return nullptr;
}

/***********************************************************************
*                METHOD DEFINITIONS                                    *
************************************************************************/

Temperature::Temperature(volatile float* ptrFeedback, int32_t threadFreq, int32_t slowUpdateFreq, const char* sensorType, const char* pinSensor, float beta, int r0, int t0) :
  Module(threadFreq, slowUpdateFreq),
  ptrFeedback(ptrFeedback),
  sensorType(sensorType),
  pinSensor(pinSensor),
	beta(beta),
	r0(r0),
	t0(t0)
{
    if (this->sensorType == "Thermistor")
    {
        this->Sensor = new Thermistor(this->pinSensor, this->beta, this->r0, this->t0);
    }
    // TODO: Add more sensor types as needed

    // Take some readings to get the ADC up and running before moving on
    this->slowUpdate();
    this->slowUpdate();
}

void Temperature::update()
{
  return;
}

void Temperature::slowUpdate()
{
	this->temperaturePV = this->Sensor->getTemperature();

    // check for disconnected temperature sensor
    if (this->temperaturePV > 0)
    {
        *(this->ptrFeedback) = this->temperaturePV;
    }
    else
    {
        *(this->ptrFeedback) = 999;
    }

}
