#include "TimePatterns/TimePattern.h"

using namespace WeightRetargeting;

patterns::TimePattern::TimePattern()
{
    lastTimestamp = std::chrono::steady_clock::now();
    lastValue = 0.0;
}

void patterns::TimePattern::update(double value)
{
    lastTimestamp = std::chrono::steady_clock::now();
    lastValue = value;
}
