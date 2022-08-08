#ifndef WEIGHT_RETARGETING_MAPPING_FUNCTION_H
#define WEIGHT_RETARGETING_MAPPING_FUNCTION_H

#include "../CommandGenerator.h"

namespace WeightRetargeting
{
    class MappingFunction;
};

class WeightRetargeting::MappingFunction : public WeightRetargeting::CommandGenerator
{
protected:
    double minThreshold;
    double maxThreshold;
public:
    void setMinThreshold(double threshold);
    void setMaxThreshold(double threshold);
};

#endif
