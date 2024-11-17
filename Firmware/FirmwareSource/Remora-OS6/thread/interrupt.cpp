#include "interrupt.h"

#if defined TARGET_STM32F4
#include "stm32f4xx.h"
#elif defined TARGET_STM32H7
#include "stm32h7xx.h"
#elif defined TARGET_STM32F1
#include "stm32f1xx.h"
#elif defined TARGET_STM32G0
#include "stm32g0xx.h"
#endif


#include <cstdio>

// Define the vector table, it is only declared in the class declaration
Interrupt* Interrupt::ISRVectorTable[] = {0};

// Constructor
Interrupt::Interrupt(void){}


// Methods

void Interrupt::Register(int interruptNumber, Interrupt* intThisPtr)
{
	ISRVectorTable[interruptNumber] = intThisPtr;
}

//void Interrupt::TIM3_Wrapper(void)
//{
//	ISRVectorTable[TIM3_IRQn]->ISR_Handler();
//}

void Interrupt::TIM3_Wrapper(void)
{
	ISRVectorTable[TIM3_IRQn]->ISR_Handler();
}

void Interrupt::TIM4_Wrapper(void)
{
	ISRVectorTable[TIM4_IRQn]->ISR_Handler();
}

void Interrupt::TIM5_Wrapper(void)
{
	ISRVectorTable[TIM5_IRQn]->ISR_Handler();
}
