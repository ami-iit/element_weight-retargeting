#ifndef WEIGHT_RETARGETING_PULSE_FEEDBACK_H
#define WEIGHT_RETARGETING_PULSE_FEEDBACK_H

#include <vector>

#include "TimePattern.h"

namespace WeightRetargeting::patterns
{
    class PulseFeedback;
};

class WeightRetargeting::patterns::PulseFeedback : public WeightRetargeting::patterns::TimePattern
{
public:
    ~PulseFeedback() = default;

    /**
     * @brief Creates pulse levels, given a maximum frequency.
     * 
     * @param levels the number of pulse levels
     * @param maxFrequency the frequency of the maximum level
     */
    void makeFrequencies(const int levels, const double maxFrequency);

    void makeFrequencies(std::vector<double> thresholds, std::vector<double> frequencies);

    void update(double value) override;
    double getCommand() override;
private:
    std::vector<double> levelThresholds;
    std::vector<double> frequencies; //TODO
};

#endif
