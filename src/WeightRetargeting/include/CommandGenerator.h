#ifndef WEIGHT_RETARGETING_COMMAND_GENERATOR_H
#define WEIGHT_RETARGETING_COMMAND_GENERATOR_H

#include <memory>

namespace WeightRetargeting{
    class CommandGenerator;
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
    void update(double value);
    virtual double getCommand() = 0;

    virtual ~CommandGenerator() = default;

};

#endif
