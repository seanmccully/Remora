// ErrorHandler.h
#ifndef MBED_ERROR_HANDLER_H
#define MBED_ERROR_HANDLER_H

#include "mbed.h"
#include "mbed_error.h"
#include <string>

// Forward declaration of the error hook
extern "C" void mbed_error_hook(const mbed_error_ctx* error_context);

class ErrorHandler {
public:
    // Singleton instance getter
    static ErrorHandler& getInstance() {
        static ErrorHandler instance;
        return instance;
    }

    // Delete copy constructor and assignment operator
    ErrorHandler(const ErrorHandler&) = delete;
    ErrorHandler& operator=(const ErrorHandler&) = delete;

    // Initialize error handling system
    void init();

    // Report custom error
    void reportError(mbed_error_status_t error_status, const char* custom_message);

    // Get last error information
    bool getLastError(mbed_error_ctx* error_info) const;

    // Clear error history
    void clearErrorHistory();

    // Perform safe system reset
    void safeSystemReset();

    // Get error count
    uint32_t getErrorCount() const { return error_count; }

    // Static callback for Mbed error hook
    static void errorCallback(const mbed_error_ctx* error_ctx);

		void handleError(const mbed_error_ctx* error_ctx);
		bool hasStoredCrashInfo() const;
		void printStoredCrashInfo() const;
		bool readCrashInfo(mbed_error_ctx* ctx) const;

private:
    // Private constructor for singleton
    ErrorHandler() {
        memset(&last_error, 0, sizeof(mbed_error_ctx));
    }


    // Convert error status to string
    static const char* errorStatusToString(mbed_error_status_t status);

    // Convert fault type to string
    static const char* faultTypeToString(uint32_t fault_type);

    // Analyze and print error context
    void analyzeErrorContext(const mbed_error_ctx* ctx) const;

    // Prepare for system reset
    void prepareForReset();

    // Member variables
    static uint32_t error_count;
    static mbed_error_ctx last_error;
};

#endif
