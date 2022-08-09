#include "ActuatorsGroupFactory.h"

#include <algorithm>

#include "MappingFunctions/StepMappingFunction.h"
#include "MappingFunctions/LinearMappingFunction.h"

using namespace WeightRetargeting;

const std::vector<std::string> ActuatorsGroupFactory::MAP_FUNCTIONS = {"linear", "steps"};

ActuatorsGroupFactory::ActuatorsGroupFactory()
{
    parseError = "";
}

/**
 * @brief Macro that sets the parse error and returns false
 */
#define ACTUATORS_GROUP_PARSE_CHECK(fail_condition, fail_message)\
                                    if(fail_condition){parseError = fail_message+"!"; return false;}

bool ActuatorsGroupFactory::parseFromConfig(yarp::os::Bottle& configGroup)
{
    const std::string groupName = configGroup.get(0).asString();

    // get map function
    std::string map_function;
    ACTUATORS_GROUP_PARSE_CHECK(!configGroup.check("map_function"), 
                                "Missing a valid map_function paramater in group " + groupName);
    map_function = configGroup.find("map_function").asString();

    ACTUATORS_GROUP_PARSE_CHECK(
        std::find(MAP_FUNCTIONS.begin(), MAP_FUNCTIONS.end(), map_function)==MAP_FUNCTIONS.end(),
        "Parameter map_function not in the accepted list of values (found " + map_function + ")");

    // get min threshold
    auto minThresholdValue = configGroup.find("min_threshold");
    ACTUATORS_GROUP_PARSE_CHECK(
        !minThresholdValue.isFloat64(),
        "Missing a valid min_threshold parameter in group " +  groupName
    )
    double minThreshold = minThresholdValue.asFloat64();

    // get max threshold
    auto maxThresholdValue = configGroup.find("max_threshold");
    ACTUATORS_GROUP_PARSE_CHECK(
        !maxThresholdValue.isFloat64(), 
        "Missing a valid max_threshold parameter in group " + groupName);
    
    double maxThreshold = maxThresholdValue.asFloat64();

    // get list of joints
    auto jointAxesValue = configGroup.find("joint_axes");
    ACTUATORS_GROUP_PARSE_CHECK(!jointAxesValue.isList(),
                                "Missing a valid joint_axes parameter in group " + groupName);
    auto jointAxesBottle = jointAxesValue.asList();
    ACTUATORS_GROUP_PARSE_CHECK(jointAxesBottle->size()==0,
                                "Empty joint_axes parameter in group " +  groupName);
    
    std::vector<std::string> jointAxes;
    for(int i=0 ; i<jointAxesBottle->size() ; i++)
    {
        jointAxes.push_back(jointAxesBottle->get(i).asString());
    }

    // get list of actuators
    auto actuatorsValue = configGroup.find("actuators");
    ACTUATORS_GROUP_PARSE_CHECK(!actuatorsValue.isList(),
                                "Missing a valid actuators parameter in group " + groupName);
    auto actuatorsBottle = actuatorsValue.asList();
    
    std::vector<std::string> actuators;
    for(int i=0 ; i<actuatorsBottle->size() ; i++)
    {
        actuators.push_back(actuatorsBottle->get(i).asString());
    }


    return true;
}

std::string& ActuatorsGroupFactory::getParseError()
{
    return parseError;
}
