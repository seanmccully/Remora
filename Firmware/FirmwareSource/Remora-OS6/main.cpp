
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
#include "data.h"
#include "pin.h"
#include "remora.h"
#include "pruThread.h"
#include "module.h"
#include "moduleList.h"

#include "mbed_error.h"
#include "mbed_fault_handler.h"
#include "errorHandler.h"
#include "File.h"
#include "SDBlockDevice.h"

#include "FATFileSystem.h"
#include "sdfile.h"

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


// Main function
int main() {
    auto& errorHandler = ErrorHandler::getInstance();
    errorHandler.init();

    Remora *remora = new Remora();
    remora->start();
}
