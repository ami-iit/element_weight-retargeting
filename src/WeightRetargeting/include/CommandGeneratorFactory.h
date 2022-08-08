#ifndef WEIGHT_RETARGETING_COMMAND_GENERATOR_FACTORY_H
#define WEIGHT_RETARGETING_COMMAND_GENERATOR_FACTORY_H

#include <yarp/os/Searchable.h>

#include "CommandGenerator.h"

namespace WeightRetargeting
{
    class CommandGeneratorFactory;
}

class WeightRetargeting::CommandGeneratorFactory
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
public:
    CommandGeneratorFactory();

    bool parseFromConfig(yarp::os::Searchable& configGroup);
    std::string& getParseError();

    CommandGenerator& getCommandGenerator();

};


#endif
