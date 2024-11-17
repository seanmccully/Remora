
#include "jsonConfigHandler.h"

JsonConfigHandler::JsonConfigHandler() : doc(MAX_JSON_SIZE)
{

    loadConfiguration();
}

bool JsonConfigHandler::loadConfiguration() {
    // Clear any existing configuration
    jsonContent.clear();
    doc.clear();

    // Read and parse the configuration file
    if (!readFileContent()) {
        return false;
    }
    parseJson();
    return true;
}


JsonArray JsonConfigHandler::getModules() {
    if (doc.containsKey("Modules"))
        return doc["Modules"].as<JsonArray>();
    else
        return JsonArray();
}

// Method to get specific module configurations
JsonObject JsonConfigHandler::getModuleConfig(const char* threadName, const char* moduleType) {
    if (!doc.containsKey("Modules")) {
        MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"",0x1243);
    }
    JsonArray modules = doc["Modules"];
    for (JsonObject module : modules) {
        if (strcmp(module["Thread"], threadName) == 0 &&
            strcmp(module["Type"], moduleType) == 0) {
            return module;
        }
    }
    return JsonObject(); // Return empty object if not found
}

bool JsonConfigHandler::readFileContent() {

		SDFile *_file = new SDFile(filename, O_RDONLY);
    // Pre-allocate string with correct size

		if (!_file->is_open()) {
        MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"",0x1243);
		}
		ssize_t _sz;
		char buffer[256];
		while (_sz = _file->read(&buffer, sizeof(buffer))) {
		 	jsonContent += buffer;
	 	}
    _file->close();

		return true;
}


bool JsonConfigHandler::parseJson() {
    // Clear any existing parsed data
    doc.clear();

    // Parse JSON
    DeserializationError error = deserializeJson(doc, jsonContent.c_str());

    if (error) {
        MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION, MBED_ERROR_CODE_INVALID_ARGUMENT),"",0x1243);
        return false;
    }

    return true;
}
