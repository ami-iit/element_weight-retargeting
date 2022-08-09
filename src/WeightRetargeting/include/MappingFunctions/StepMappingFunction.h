#ifndef WEIGHT_RETARGETING_STEP_MAPPING_H
#define WEIGHT_RETARGETING_STEP_MAPPING_H

#include <vector>

#include "LinearMappingFunction.h"

namespace WeightRetargeting::mapping
{
    class StepMappingFunction;
}

class WeightRetargeting::mapping::StepMappingFunction : public WeightRetargeting::mapping::LinearMappingFunction
{
private:
    std::vector<double> thresholds;
    std::vector<double> commands;
public:
    StepMappingFunction();

    /**
     * @brief create equally distributed value and command steps
     * 
     * @param n the number of steps
     */
    void makeSteps(const int n);

    /**
     * @brief create command steps
     * 
     * @param thresholds the input steps to decide the threshold
     * @param commands the command values associated to the steps
     */
    void makeSteps(const std::vector<double>& thresholds, const std::vector<double>& commands);

    double getCommand() override;
};


#endif
