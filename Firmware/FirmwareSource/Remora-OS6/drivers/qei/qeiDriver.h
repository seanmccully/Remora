#ifndef QEIDRIVER_H
#define QEIDRIVER_H

#include "mbed.h"

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <string>

#if defined TARGET_STM32F4
#include "stm32f4xx.h"
#elif defined TARGET_STM32H7
#include "stm32h7xx.h"
#elif defined TARGET_STM32F1
#include "stm32f1xx.h"
#elif defined TARGET_STM32G0
#include "stm32g0xx.h"
#endif


class QEIdriver
{
    private:

        TIM_HandleTypeDef       htim;
        TIM_Encoder_InitTypeDef sConfig  = {0};
        TIM_MasterConfigTypeDef sMasterConfig  = {0};

        InterruptIn             qeiIndex;
        IRQn_Type 		        irq;

        void interruptHandler();

    public:

        bool                    hasIndex;
        bool                    indexDetected;
        int32_t                 indexCount;

        QEIdriver();            // for channel A & B
        QEIdriver(bool);        // For channels A & B, and index

        void init(void);
        uint32_t get(void);

};

#endif
