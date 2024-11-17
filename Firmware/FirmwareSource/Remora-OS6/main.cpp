
/*
Remora PRU firmware for LinuxCNC
Copyright (C) 2021  Scott Alford (scotta)

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License version 2
of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

// MBED includes
#include "mbed.h"
#include <cstdio>
#include <cerrno>
#include <string>
#include <vector>
#include <memory>
#include <map>

#include "board.h"
#include "pin.h"
#include "remora.h"
#include "pruThread.h"
#include "module.h"
#include "moduleList.h"
#include "arm_irq.h"

#include "mbed_error.h"
#include "mbed_fault_handler.h"
#include "errorHandler.h"
#include "File.h"

#include "FATFileSystem.h"
#include "sdfile.h"
#include "RemoraCAN.h"
#include "data.h"

#include <string>
#include <sstream>


// Example usage
/***********************************************************************
        OBJECTS etc
************************************************************************/
#if MBED_CONF_PLATFORM_CRASH_CAPTURE_ENABLED

mbed_error_status_t err_status;
mbed_fault_context_t fault_ctx;
void mbed_error_hook(const mbed_error_ctx *error_context)
{
    //Dont do anything here, let application override this if required.
    ErrorHandler::getInstance().handleError(error_context);
}

void mbed_error_reboot_callback(mbed_error_ctx *error_context)
{
    //Dont do anything here, let application override this if required.
    ErrorHandler::getInstance().handleError(error_context);
}

#endif


/***********************************************************************
        INTERRUPT HANDLERS - add NVIC_SetVector etc to setup()
************************************************************************/

// Add these to /thread/irqHandlers.h in the TARGET_target


/***********************************************************************
        ROUTINES
************************************************************************/
void irq_disable(void)
{
    asm volatile("cpsid i" ::: "memory");
}

void irq_enable(void)
{
    asm volatile("cpsie i" ::: "memory");
}

irqstatus_t irq_save(void)
{
    irqstatus_t flag;
    asm volatile("mrs %0, primask" : "=r" (flag) :: "memory");
    irq_disable();
    return flag;
}

void irq_restore(irqstatus_t flag)
{
    asm volatile("msr primask, %0" :: "r" (flag) : "memory");
}

void irq_wait(void)
{
    if (__CORTEX_M >= 7)
        // Cortex-m7 may disable cpu counter on wfi, so use nop
        asm volatile("cpsie i\n    nop\n    cpsid i\n" ::: "memory");
    else
        asm volatile("cpsie i\n    wfi\n    cpsid i\n" ::: "memory");
}

void irq_poll(void)
{
}

// Global Data Buffers
volatile rxData_t* rxData = new rxData_t();
volatile txData_t* txData = new txData_t();
// pointers to data
volatile uint32_t* ptrTxHeader = &txData->header;
volatile bool*    ptrPRUreset = &PRUreset;
volatile uint32_t* ptrJointFreqCmd = &rxData->jointFreqCmd[0];
volatile uint32_t* ptrJointFeedback = &txData->jointFeedback[0];
volatile uint8_t* ptrJointEnable = &rxData->jointEnable;
volatile float*   ptrSetPoint = &rxData->setPoint[0];
volatile float*   ptrProcessVariable = &txData->processVariable[0];
volatile uint16_t* ptrInputs = &txData->inputs;
volatile uint16_t* ptrOutputs = &rxData->outputs;



// Main function
int main() {
    init_board_pins();

    Remora *remora = new Remora();
    while (!remora->isError()) {
        remora->handleState(); // should be called at least twice;
    }
}
