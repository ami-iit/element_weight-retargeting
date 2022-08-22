#include "ActuatorsGroupFactory.h"

#include <algorithm>

#include "MappingFunctions/StepMappingFunction.h"
#include "MappingFunctions/LinearMappingFunction.h"

#include "TimePatterns/ContinuousFeedback.h"
#include "TimePatterns/PulseFeedback.h"

using namespace WeightRetargeting;

const std::string ActuatorsGroupFactory::LINEAR_MAP_FUNCTION_NAME = "linear";
const std::string ActuatorsGroupFactory::STEP_MAP_FUNCTION_NAME = "steps";
const std::vector<std::string> ActuatorsGroupFactory::MAP_FUNCTIONS = {LINEAR_MAP_FUNCTION_NAME, STEP_MAP_FUNCTION_NAME};


const std::string ActuatorsGroupFactory::CONTINUOUS_TIME_PATTERN_NAME = "continuous";
const std::string ActuatorsGroupFactory::PULSE_TIME_PATTERN_NAME = "pulse";
const std::vector<std::string> ActuatorsGroupFactory::TIME_PATTERNS = {CONTINUOUS_TIME_PATTERN_NAME, PULSE_TIME_PATTERN_NAME};


ActuatorsGroupFactory::ActuatorsGroupFactory()
{
    parseError = "";
}

/**
 * @brief Macro that sets the parse error and returns false
 */
#define ACTUATORS_GROUP_PARSE_CHECK(fail_condition, fail_message)\
                                    if(fail_condition){parseError = fail_message+"!"; return false;}


bool ActuatorsGroupFactory::parseStepMapFunction(yarp::os::Bottle& configGroup)
{
    stepFunctionNumber = -1; //flag for "not found"
    stepFunctionThresholds.clear();
    stepFunctionCommands.clear();

    if(configGroup.check("steps_number"))
    {
        ACTUATORS_GROUP_PARSE_CHECK(!configGroup.find("steps_number").isInt32(), 
                            "Invalid steps_number paramater in group " + groupName)

        stepFunctionNumber = configGroup.find("steps_number").asInt32();
        return true;
    }

    if(configGroup.check("steps_thresholds") && configGroup.check("steps_commands"))
    {
        ACTUATORS_GROUP_PARSE_CHECK(!configGroup.find("steps_thresholds").isList(), 
                            "Invalid steps_thresholds paramater in group " + groupName)

        ACTUATORS_GROUP_PARSE_CHECK(!configGroup.find("steps_commands").isList(), 
                            "Invalid steps_commands paramater in group " + groupName)

        auto stepThresholdsBottle = configGroup.find("steps_thresholds").asList();
        auto stepCommandsBottle = configGroup.find("steps_commands").asList();

        ACTUATORS_GROUP_PARSE_CHECK(stepCommandsBottle->size()!=stepThresholdsBottle->size()+1, 
                            "The size of steps_commands paramater must be the size of step_thresholds+1 in group " + groupName)

        for(int i=0; i< stepThresholdsBottle->size() ; i++)
        {
            stepFunctionThresholds.push_back(stepThresholdsBottle->get(i).asFloat64());
        }

        for(int i=0; i< stepCommandsBottle->size() ; i++)
        {
            stepFunctionCommands.push_back(stepCommandsBottle->get(i).asFloat64());
        }

        return true;
    }

    return false;
}

bool ActuatorsGroupFactory::parseMapFunction(yarp::os::Bottle& configGroup)
{
    // get min threshold
    auto minThresholdValue = configGroup.find("min_threshold");
    ACTUATORS_GROUP_PARSE_CHECK(
        !minThresholdValue.isFloat64(),
        "Missing a valid min_threshold parameter in group " +  groupName)
    minThreshold = minThresholdValue.asFloat64();

    // get max threshold
    auto maxThresholdValue = configGroup.find("max_threshold");
    ACTUATORS_GROUP_PARSE_CHECK(
        !maxThresholdValue.isFloat64(), 
        "Missing a valid max_threshold parameter in group " + groupName)
    maxThreshold = maxThresholdValue.asFloat64();

    // get map function
    ACTUATORS_GROUP_PARSE_CHECK(!configGroup.check("map_function"), 
                                "Missing a valid map_function paramater in group " + groupName)
    mapFunction = configGroup.find("map_function").asString();

    ACTUATORS_GROUP_PARSE_CHECK(
        std::find(MAP_FUNCTIONS.begin(), MAP_FUNCTIONS.end(), mapFunction)==MAP_FUNCTIONS.end(),
        "Parameter map_function not in the accepted list of values (found " + mapFunction + ")")

    if(mapFunction==STEP_MAP_FUNCTION_NAME && !parseStepMapFunction(configGroup))
    {
        return false;
    }

    return true;
}

bool ActuatorsGroupFactory::parsePulseTimePattern(yarp::os::Bottle& configGroup)
{
    // reset related variables
    pulsePatternLevels = -1;
    pulsePatternMaxFrequency = 0.;
    pulsePatternFrequencies.clear();
    pulsePatternThresholds.clear();

    // case 1 - auto generate levels
    bool pulsePatternLevelsFound = configGroup.check("pulse_pattern_levels");
    bool pulsePatternMaxFreqFound = configGroup.check("pulse_pattern_max_frequency");
    
    ACTUATORS_GROUP_PARSE_CHECK(pulsePatternLevelsFound!=pulsePatternMaxFreqFound, 
                            "Provide both parameters pulse_pattern_levels and pulse_pattern_max_frequency or neither of them in " + groupName)

    if(pulsePatternLevelsFound)
    {
        pulsePatternLevels = configGroup.find("pulse_pattern_levels").asInt32();
        pulsePatternMaxFrequency = configGroup.find("pulse_pattern_max_frequency").asFloat64();
    }

    //case 2 - explicit levels and frequencies
    bool pulsePatternThresholdsFound = configGroup.check("pulse_pattern_thresholds");
    bool pulsePatternFrequenciesFound = configGroup.check("pulse_pattern_frequencies");

    ACTUATORS_GROUP_PARSE_CHECK(pulsePatternThresholdsFound!=pulsePatternFrequenciesFound, 
                            "Provide both parameters pulse_pattern_thresholds and pulse_pattern_frequencies or neither of them in group " + groupName)

    if(pulsePatternThresholdsFound)
    {
        auto pulsePatternThresholdsList = configGroup.find("pulse_pattern_thresholds").asList();
        auto pulsePatternFrequenciesList = configGroup.find("pulse_pattern_frequencies").asList();

        ACTUATORS_GROUP_PARSE_CHECK(pulsePatternThresholdsList==nullptr,
                                "Parameter pulse_pattern_thresholds is not a list in group " + groupName)

        ACTUATORS_GROUP_PARSE_CHECK(pulsePatternFrequenciesList==nullptr, 
                                "Parameter pulse_pattern_frequencies is not a list in group " + groupName)
        
        ACTUATORS_GROUP_PARSE_CHECK(pulsePatternThresholdsList->size()!=pulsePatternFrequenciesList->size(), 
                                "Parameters pulse_pattern_thresholds and pulse_pattern_frequencies must have the same size in group " + groupName)

        for(int i = 0; i<pulsePatternThresholdsList->size(); i++)
        {
            pulsePatternThresholds.push_back(pulsePatternThresholdsList->get(i).asFloat64());
            pulsePatternFrequencies.push_back(pulsePatternFrequenciesList->get(i).asFloat64());
        }
        
    }

    ACTUATORS_GROUP_PARSE_CHECK(pulsePatternLevelsFound && pulsePatternThresholdsFound || !pulsePatternLevelsFound && !pulsePatternThresholdsFound, 
                                "Provide one between pulse_pattern_levels and pulse_pattern_thresholds and not both in group " + groupName)

    
    // get pulse custom actuation
    pulsePatternCustomActuation = -1.;
    if(configGroup.check("pulse_pattern_custom_actuation"))
    {
        pulsePatternCustomActuation = configGroup.find("pulse_pattern_custom_actuation").asFloat64();
    }

    return true;
}

bool ActuatorsGroupFactory::parseTimePattern(yarp::os::Bottle& configGroup)
{
    // get time pattern function
    ACTUATORS_GROUP_PARSE_CHECK(!configGroup.check("time_pattern"), 
                                "Missing a valid time_pattern paramater in group " + groupName)
    timePattern = configGroup.find("time_pattern").asString();

    ACTUATORS_GROUP_PARSE_CHECK(
        std::find(TIME_PATTERNS.begin(), TIME_PATTERNS.end(), timePattern)==TIME_PATTERNS.end(),
        "Parameter time_pattern not in the accepted list of values (found " + timePattern + ")")

    if(timePattern==PULSE_TIME_PATTERN_NAME && !parsePulseTimePattern(configGroup))
    {
        return false;
    }
    
    return true;
}

bool ActuatorsGroupFactory::parseFromConfig(yarp::os::Bottle& configGroup)
{
    groupName = configGroup.get(0).asString();

    // get map function
    if(!parseMapFunction(configGroup))
    {
        return false;
    }

    // get time pattern
    if(!parseTimePattern(configGroup))
    {
        return false;
    }

    // get list of joints
    auto jointAxesValue = configGroup.find("joint_axes");
    ACTUATORS_GROUP_PARSE_CHECK(!jointAxesValue.isList(),
                                "Missing a valid joint_axes parameter in group " + groupName);
    auto jointAxesBottle = jointAxesValue.asList();
    ACTUATORS_GROUP_PARSE_CHECK(jointAxesBottle->size()==0,
                                "Empty joint_axes parameter in group " +  groupName);
    
    jointAxes.clear();
    for(int i=0 ; i<jointAxesBottle->size() ; i++)
    {
        jointAxes.push_back(jointAxesBottle->get(i).asString());
    }

    // get list of actuators
    auto actuatorsValue = configGroup.find("actuators");
    ACTUATORS_GROUP_PARSE_CHECK(!actuatorsValue.isList(),
                                "Missing a valid actuators parameter in group " + groupName);
    auto actuatorsBottle = actuatorsValue.asList();
    
    actuators.clear();
    for(int i=0 ; i<actuatorsBottle->size() ; i++)
    {
        actuators.push_back(actuatorsBottle->get(i).asString());
    }

    makeGroup();
    return true;
}

void ActuatorsGroupFactory::makeGroup()
{
    parsedActuatorsGroups.emplace(groupName, ActuatorsGroup());
    auto& actuatorGroup = parsedActuatorsGroups.at(groupName);

    actuatorGroup.name = groupName;
    actuatorGroup.jointAxes = jointAxes;
    actuatorGroup.actuators = actuators;
    actuatorGroup.minThreshold = minThreshold;
    actuatorGroup.maxThreshold = maxThreshold;

    // map function
    if(mapFunction==LINEAR_MAP_FUNCTION_NAME)
    {
        mapping::LinearMappingFunction *mapFunctionPtr = new mapping::LinearMappingFunction;

        mapFunctionPtr->setMaxThreshold(maxThreshold);
        mapFunctionPtr->setMinThreshold(minThreshold);

        actuatorGroup.mapFunction.reset(mapFunctionPtr);
    }
    else if(mapFunction==STEP_MAP_FUNCTION_NAME)
    {
        mapping::StepMappingFunction *mapFunctionPtr = new mapping::StepMappingFunction;

        mapFunctionPtr->setMaxThreshold(maxThreshold);
        mapFunctionPtr->setMinThreshold(minThreshold);
        
        if(stepFunctionNumber!=-1)
        {
            mapFunctionPtr->makeSteps(stepFunctionNumber);
        }
        else
        {
            mapFunctionPtr->makeSteps(stepFunctionThresholds, stepFunctionCommands);
        }


        actuatorGroup.mapFunction.reset(mapFunctionPtr);
    }

    if(timePattern==CONTINUOUS_TIME_PATTERN_NAME)
    {
        actuatorGroup.timePattern.reset(new patterns::ContinuousFeedback);
    }
    else if(timePattern==PULSE_TIME_PATTERN_NAME)
    {
        patterns::PulseFeedback* pulseFeedback = new patterns::PulseFeedback;

        if(pulsePatternLevels!=-1)
        {
            pulseFeedback->makeFrequencies(pulsePatternLevels, pulsePatternMaxFrequency);
        }
        else
        {
            pulseFeedback->makeFrequencies(pulsePatternThresholds, pulsePatternFrequencies);
        }

        pulseFeedback->setCustomActuation(pulsePatternCustomActuation);

        actuatorGroup.timePattern.reset(pulseFeedback);
    }
}

WeightRetargeting::ActuatorsGroup& ActuatorsGroupFactory::getGroup(std::string& name)
{
    return parsedActuatorsGroups.at(groupName);
}

std::string& ActuatorsGroupFactory::getParseError()
{
    return parseError;
}
