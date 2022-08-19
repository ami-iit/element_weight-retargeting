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
    double minThreshold;
    double maxThreshold;
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
    int stepFunctionNumber; // number of steps for the step function
    std::vector<double> stepFunctionThresholds; // step function custom thresholds
    std::vector<double> stepFunctionCommands; // step function custom commands
    std::string timePattern; // name of the time pattern function
    int pulsePatternLevels; // levels for the pulse pattern
    double pulsePatternMaxFrequency; // frequency of the highest level for pulse pattern
    std::vector<double> pulsePatternThresholds; // thresholds for the levels of the pulse pattern
    std::vector<double> pulsePatternFrequencies; // frequencies for the levels of the pulse pattern

    // parsed structures
    std::unordered_map<std::string, ActuatorsGroup> parsedActuatorsGroups;

    // parse sub-methods
    bool parseMapFunction(yarp::os::Bottle& configGroup);
    bool parseStepMapFunction(yarp::os::Bottle& configGroup);
    bool parseTimePattern(yarp::os::Bottle& configGroup);
    bool parsePulseTimePattern(yarp::os::Bottle& configGroup);

    // creates the group
    void makeGroup();
public:
    // value mapping functions
    const static std::vector<std::string> MAP_FUNCTIONS;
    const static std::string LINEAR_MAP_FUNCTION_NAME;
    const static std::string STEP_MAP_FUNCTION_NAME;

    // time patterns
    const static std::vector<std::string> TIME_PATTERNS;
    const static std::string CONTINUOUS_TIME_PATTERN_NAME;
    const static std::string PULSE_TIME_PATTERN_NAME;

    ActuatorsGroupFactory();

    bool parseFromConfig(yarp::os::Bottle& configGroup);
    std::string& getParseError();

    WeightRetargeting::ActuatorsGroup& getGroup(std::string& name);
};


#endif
