#include "MappingFunctions/MappingFunction.h"

using namespace WeightRetargeting::mapping;

void MappingFunction::setMaxThreshold(double threshold)
{
    this->maxThreshold = threshold;
}

void MappingFunction::setMinThreshold(double threshold)
{
    this->minThreshold = threshold;
}
