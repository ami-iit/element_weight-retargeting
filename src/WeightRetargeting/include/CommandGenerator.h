#ifndef WEIGHT_RETARGETING_COMMAND_GENERATOR_H
#define WEIGHT_RETARGETING_COMMAND_GENERATOR_H

#include <memory>
#include <vector>

namespace WeightRetargeting{
    class CommandGenerator;
    class CommandGeneratorChain;
}

class WeightRetargeting::CommandGenerator
{
protected:
    double lastValue;

public:
    /**
     * @brief Update the state of the command generator with the given value 
     * 
     * @param value 
     */
    virtual void update(double value);
    virtual double getCommand() = 0;

    virtual ~CommandGenerator() = default;

};


class WeightRetargeting::CommandGeneratorChain : public CommandGenerator
{
public:
    virtual void update(double value) override;
    virtual double getCommand() override;

    virtual ~CommandGeneratorChain() = default;

    void addGenerator(CommandGenerator* generator);
private:
    std::vector<std::unique_ptr<CommandGenerator>> generators;
};

#endif
