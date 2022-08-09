#ifndef WEIGHT_RETARGETING_LINEAR_MAPPING_H
#define WEIGHT_RETARGETING_LINEAR_MAPPING_H

#include "MappingFunction.h"

namespace WeightRetargeting::mapping
{
    class LinearMappingFunction;
}

class WeightRetargeting::mapping::LinearMappingFunction : public WeightRetargeting::mapping::MappingFunction
{
public:
    double getCommand() override;
};

#endif
