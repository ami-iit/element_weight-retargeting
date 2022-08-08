#include "CommandGeneratorFactory.h"

using namespace WeightRetargeting;

class CommandGeneratorFactory::Impl
{

};

CommandGeneratorFactory::CommandGeneratorFactory()
{
    pImpl = std::make_unique<CommandGeneratorFactory::Impl>();
}
