#ifndef WEIGHT_RETARGETING_CONTINUOUS_FEEDBACK_H
#define WEIGHT_RETARGETING_CONTINUOUS_FEEDBACK_H

#include "TimePattern.h"

namespace WeightRetargeting::patterns
{
    class ContinuousFeedback;
};

class WeightRetargeting::patterns::ContinuousFeedback : public WeightRetargeting::patterns::TimePattern
{
    double getCommand() override;
};

#endif
