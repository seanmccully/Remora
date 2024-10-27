#ifndef MODULE_FACTORY_H
#define MODULE_FACTORY_H

#include <algorithm>
#include <functional>
#include <memory>
#include <unordered_map>
#include <string>

#include "module.h"
#include "jsonConfigHandler.h"

using namespace std;

typedef  unique_ptr<Module> (*ModuleCreator)(const JsonObject&);
class ModuleFactory {
private:

    ModuleCreator createOnLoadModule(const char* modN);
    ModuleCreator createServoModule(const char* modN);
    ModuleCreator createBaseModule(const char* modN);
    //unordered_map<const char*, unordered_map<const char*, ModuleCreator>> moduleCreators;

    // Private constructor for singleton
    ModuleFactory() {};

public:
    static ModuleFactory* getInstance();
    // Create module based on thread and type
    std::unique_ptr<Module> createModule(const char* _tname, 
                                       const char* _ttype,
                                       const JsonVariant config);

};

#endif
