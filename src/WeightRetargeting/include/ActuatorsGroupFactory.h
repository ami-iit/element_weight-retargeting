#ifndef WEIGHT_RETARGETING_ACTUATORS_GROUP_FACTORY_H
#define WEIGHT_RETARGETING_ACTUATORS_GROUP_FACTORY_H

#include <vector>
#include <unordered_map>

#include <yarp/os/Bottle.h>

#include "CommandGenerator.h"

namespace WeightRetargeting
{
    class ActuatorsGroup;
    class ActuatorsGroupFactory;
}

struct WeightRetargeting::ActuatorsGroup
{
    std::string name;
    std::vector<std::string> jointAxes;
    std::unique_ptr<CommandGenerator> commandGenerator;
    std::vector<std::string> actuators;
};

class WeightRetargeting::ActuatorsGroupFactory
{
private:
    // parse utilities
    std::string parseError;
    // buffer variables
    std::string groupName;
    std::string mapFunction;
    double minThreshold;
    double maxThreshold;
    std::vector<std::string> jointAxes;
    std::vector<std::string> actuators;

    // parsed structures
    std::unordered_map<std::string, ActuatorsGroup> parsedActuatorsGroups;


    // creates the group
    void makeGroup();
public:
    const static std::vector<std::string> MAP_FUNCTIONS;

    ActuatorsGroupFactory();

    bool parseFromConfig(yarp::os::Bottle& configGroup);
    std::string& getParseError();

   //TODO how to return the infos
};


#endif
