#ifndef QEI_H
#define QEI_H

#include "mbed.h"
#include <cstdint>

#include "module.h"
#include "qeiDriver.h"

#include "remora.h"

unique_ptr<Module> createQEI(const JsonObject& config);

class QEI : public Module
{

	private:

        QEIdriver*              qei;

        volatile uint16_t*      ptrData; 	// pointer to the data source
		int                     bitNumber;				// location in the data source
        int                     mask;

		volatile float*         ptrEncoderCount; 	// pointer to the data source

        bool                    hasIndex;
        int32_t                 count;
        int8_t                  indexPulse;
        int8_t                  pulseCount;

	public:

        QEI(volatile float&);                           // for channel A & B
        QEI(volatile float&, volatile uint16_t&, int);   // For channels A & B, and index

		virtual void update(void);
};

#endif
