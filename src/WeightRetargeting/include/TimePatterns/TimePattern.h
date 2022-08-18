#ifndef WEIGHT_RETARGETING_TIME_PATTERN_H
#define WEIGHT_RETARGETING_TIME_PATTERN_H

#include "../CommandGenerator.h"

#include <chrono>

namespace WeightRetargeting::patterns
{
    class TimePattern;
};

class WeightRetargeting::patterns::TimePattern : public WeightRetargeting::CommandGenerator
{
public:
    TimePattern();
    virtual ~TimePattern() = default;

    virtual void update(double value) override;
    virtual double getCommand() = 0;

protected:
    std::chrono::steady_clock::time_point lastTimestamp;
    double lastValue;
};

#endif
