// Add these to your header file or at the top of your main.cpp
#ifndef JSON_CONFIG_HANDLER_H
#define JSON_CONFIG_HANDLER_H
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include "ArduinoJson.h"
#include "mbed.h"

#include "FATFileSystem.h"
#include "SDMMCBlockDevice.h"
#include "sdfile.h"

class JsonConfigHandler {
private:
    static constexpr size_t MAX_JSON_SIZE = 16384; // 16KB max config size
    static constexpr size_t READ_BUFFER_SIZE = 256;
    std::string jsonContent = "";
    const char* filename = "config.txt";
    DynamicJsonDocument doc;
    bool readFileContent();
    bool parseJson();


public:
    JsonConfigHandler();
    bool loadConfiguration();
    JsonArray getModules();
    JsonObject getModuleConfig(const char* threadName, const char* moduleType);

};
#endif
