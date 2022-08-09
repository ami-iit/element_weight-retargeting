#include "MappingFunctions/LinearMappingFunction.h"

using namespace WeightRetargeting::mapping;

double LinearMappingFunction::getCommand()
{
    double retValue = 0.0;
    double normalizedValue = (lastValue - minThreshold) / (maxThreshold - minThreshold);
    if(normalizedValue>0)
    {
        if(normalizedValue>1.0)
            normalizedValue = 1.0;

        retValue = normalizedValue;
    }

    return retValue;
}
