#include "mbed.h"

#include <iostream>
#include <stdio.h>

#include "interrupt.h"
#include "timerInterrupt.h"
#include "timer.h"
#include "pruThread.h"



// Timer constructor
pruTimer::pruTimer(TIM_TypeDef* timer, IRQn_Type irq, uint32_t frequency, pruThread* ownerPtr):
	timer(timer),
	irq(irq),
	frequency(frequency),
	timerOwnerPtr(ownerPtr)
{
	interruptPtr = new TimerInterrupt(this->irq, this);	// Instantiate a new Timer Interrupt object and pass "this" pointer

	this->startTimer();
}


void pruTimer::timerTick(void)
{
	//Do something here
	this->timerOwnerPtr->update();
}



void pruTimer::startTimer(void)
{
    uint32_t TIM_CLK;

    if (this->timer == TIM3)
    {
        __TIM3_CLK_ENABLE();
        TIM_CLK = APB1CLK;
    }
    else if (this->timer == TIM4)
    {
        __TIM4_CLK_ENABLE();
        TIM_CLK = APB1CLK;
    }
    else if (this->timer == TIM5)
    {
        __TIM5_CLK_ENABLE();
        TIM_CLK = APB1CLK;
    }
    /**
    else if (this->timer == TIM11)
    {
        __TIM11_CLK_ENABLE();
        TIM_CLK = APB1CLK;
    }
*/
    //timer uptade frequency = TIM_CLK/(TIM_PSC+1)/(TIM_ARR + 1)

    this->timer->CR2 &= 0;                                            // UG used as trigg output
    this->timer->PSC = TIM_PSC-1;                                     // prescaler
    this->timer->ARR = ((TIM_CLK / TIM_PSC / this->frequency) - 1);   // period           
    this->timer->EGR = TIM_EGR_UG;                                    // reinit the counter
    this->timer->DIER = TIM_DIER_UIE;                                 // enable update interrupts

    this->timer->CR1 |= TIM_CR1_CEN;                                  // enable timer

    NVIC_EnableIRQ(this->irq);
}

void pruTimer::stopTimer()
{
    NVIC_DisableIRQ(this->irq);

    this->timer->CR1 &= (~(TIM_CR1_CEN));     // disable timer
}
