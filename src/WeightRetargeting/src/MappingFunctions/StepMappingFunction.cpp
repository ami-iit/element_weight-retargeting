#include "MappingFunctions/StepMappingFunction.h"

#include<algorithm>
#include<functional>
#include<exception>

using namespace WeightRetargeting::mapping;

StepMappingFunction::StepMappingFunction() : LinearMappingFunction(){};

void StepMappingFunction::makeSteps(const std::vector<double>& thresholds, const std::vector<double>& commands)
{
    if(commands.size()!=thresholds.size()+1)
        throw std::invalid_argument("Commands must have thresholds.size()+1 values");

    if(!std::is_sorted(thresholds.begin(), thresholds.end()))
        throw std::invalid_argument("Thresholds parameter must be a sorted list of values");

    this->commands.clear();
    this->thresholds.clear();

    for(double threshold : thresholds)
    {
        this->thresholds.push_back( (maxThreshold - threshold) / (maxThreshold - minThreshold));
    }
    this->commands = commands;
}

void StepMappingFunction::makeSteps(const int n)
{
    commands.clear();
    thresholds.clear();
    commands.push_back(0.0);
    for(int i=1; i<=n; i++)
    {
        thresholds.push_back(i/(double)n);
        commands.push_back(i/(double)n);
    }

}


double StepMappingFunction::getCommand()
{
    //TODO TODO TODO check this
    double normalizedValue = LinearMappingFunction::getCommand();

    for(int i = thresholds.size()-1; i>=0 ; i--)
    {
        if(normalizedValue>=thresholds[i])
        {
            return commands[i+1];
        }
    }

    return 0;
}
