#include "CommandGenerator.h"

using namespace WeightRetargeting;


void CommandGenerator::update(double value)
{
    this->lastValue = value;
}


void CommandGeneratorChain::update(double value)
{
    CommandGenerator::update(value);

    double propagatedValue = value;

    for(const auto& generator : generators)
    {
        generator->update(propagatedValue);
        propagatedValue = generator->getCommand();
    }
}

double CommandGeneratorChain::getCommand()
{
    return generators[generators.size()-1]->getCommand();
}

void CommandGeneratorChain::addGenerator(CommandGenerator* generator)
{
    std::unique_ptr<CommandGenerator> ptr = nullptr;
    ptr.reset(generator);
    
    generators.push_back(std::move(ptr));
}
