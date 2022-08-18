#include "TimePatterns/ContinuousFeedback.h"

using namespace WeightRetargeting;

double patterns::ContinuousFeedback::getCommand()
{
    return lastValue;
}
