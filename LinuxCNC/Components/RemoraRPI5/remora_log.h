#ifndef REMORA_LOG_H
#define REMORA_LOG_H

#include "rtapi.h"
#include <stdarg.h>

// Log levels that match RTAPI message levels
typedef enum {
    LOG_ERROR   = RTAPI_MSG_ERR,     // Serious errors
    LOG_WARNING = RTAPI_MSG_WARN,    // Warning messages
    LOG_INFO    = RTAPI_MSG_INFO,    // Informational messages
    LOG_DEBUG   = RTAPI_MSG_DBG,     // Debug messages
    LOG_ALL     = RTAPI_MSG_ALL      // All messages
} log_level_t;

// Component name for logging
static const char* LOG_COMPONENT = "REMORA";

// Global debug level - can be changed at runtime
static int current_debug_level = LOG_INFO;

// Wrapper function for rtapi_print_msg with variable arguments
static void remora_log(log_level_t level, const char* format, ...) {
    // Only print if message level is less than or equal to current debug level
    if (level <= current_debug_level) {
        char buffer[512];  // Temporary buffer for formatted message
        va_list args;
        
        // Format component prefix
        int prefix_len = snprintf(buffer, sizeof(buffer), "%s: ", LOG_COMPONENT);
        
        // Format the rest of the message
        va_start(args, format);
        vsnprintf(buffer + prefix_len, sizeof(buffer) - prefix_len, format, args);
        va_end(args);
        
        // Send to RTAPI logging system
        rtapi_print_msg(level, "%s", buffer);
    }
}

// Set the debug level
static void remora_set_debug_level(log_level_t level) {
    current_debug_level = level;
    remora_log(LOG_INFO, "Debug level set to %d\n", level);
}

// Convenience macros for different log levels
#define log_error(...)   remora_log(LOG_ERROR, __VA_ARGS__)
#define log_warning(...) remora_log(LOG_WARNING, __VA_ARGS__)
#define log_info(...)    remora_log(LOG_INFO, __VA_ARGS__)
#define log_debug(...)   remora_log(LOG_DEBUG, __VA_ARGS__)

// Function result logging helper
static int log_result(const char* operation, int result) {
    if (result < 0) {
        log_error("%s failed: %s\n", operation, strerror(errno));
    } else if (result == 0) {
        log_warning("%s completed with warning\n", operation);
    } else {
        log_debug("%s successful\n", operation);
    }
    return result;
}


#endif // REMORA_LOG_H
