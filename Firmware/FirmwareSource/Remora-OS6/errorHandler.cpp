// ErrorHandler.cpp
#include "errorHandler.h"
#include "mbed_error.h"

uint32_t ErrorHandler::error_count = 0;
mbed_error_ctx ErrorHandler::last_error = {};

void ErrorHandler::handleError(const mbed_error_ctx* error_ctx) {
    error_count++;
    
    // Store the last error
    memcpy(&last_error, error_ctx, sizeof(mbed_error_ctx));
    
    // Analyze error
    analyzeErrorContext(error_ctx);
}

bool ErrorHandler::hasStoredCrashInfo() const {
    mbed_error_ctx ctx;
    return readCrashInfo(&ctx);
}

bool ErrorHandler::readCrashInfo(mbed_error_ctx* ctx) const {
    if (!ctx) return false;
    
    // Try to get the first error info from crash data region
    mbed_error_status_t status = mbed_get_first_error_info(ctx);
    return (status == MBED_SUCCESS);
}

void ErrorHandler::printStoredCrashInfo() const {
    mbed_error_ctx ctx;
    if (readCrashInfo(&ctx)) {
        //("\r\n=== Stored Crash Information ===\r\n");
        analyzeErrorContext(&ctx);
    }
}

const char* ErrorHandler::errorStatusToString(mbed_error_status_t status) {
	return "Unknown Error";
}

const char* ErrorHandler::faultTypeToString(uint32_t fault_type) {
	return "Unknown Fault";
}

void ErrorHandler::analyzeErrorContext(const mbed_error_ctx* ctx) const {
    if (!ctx) return;
    return; // printf causing HardFaults TODO;
}

void ErrorHandler::init() {
    
    // Check for stored crash information
    if (hasStoredCrashInfo()) {
        printStoredCrashInfo();
        
        // Optionally clear the crash info after reading
        // clearErrorHistory();
    }
}

void ErrorHandler::reportError(mbed_error_status_t error_status, const char* custom_message) {
    MBED_ERROR(error_status, custom_message);
}

bool ErrorHandler::getLastError(mbed_error_ctx* error_info) const {
    if (error_count == 0) {
        return false;
    }
    memcpy(error_info, &last_error, sizeof(mbed_error_ctx));
    return true;
}

void ErrorHandler::clearErrorHistory() {
    error_count = 0;
    memset(&last_error, 0, sizeof(mbed_error_ctx));
    mbed_clear_all_errors();
}

void ErrorHandler::prepareForReset() {}

void ErrorHandler::safeSystemReset() {
    prepareForReset();
    NVIC_SystemReset();
}
