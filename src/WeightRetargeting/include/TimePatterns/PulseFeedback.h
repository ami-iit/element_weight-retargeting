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
    PulseFeedback();
    ~PulseFeedback() = default;

    /**
     * @brief Creates pulse levels, given a maximum frequency.
     * 
     * @param levels the number of pulse levels
     * @param maxFrequency the frequency of the maximum level
     */
    void makeFrequencies(const int levels, const double maxFrequency);

    void makeFrequencies(std::vector<double> thresholds, std::vector<double> frequencies);

    /**
     * @brief Set a custom actuation value for the pulses.
     * 
     * @param customActuation the value of the custom actuation. If negative, the input value will be used as output.
     */
    void setCustomActuation(const double customActuation);

    void update(double value) override;
    double getCommand() override;
private:
    // configuration
    std::vector<double> levelThresholds;
    std::vector<double> periods; //TODO
    double customActuation = -1.;

    // state
    int level;
    std::chrono::steady_clock::time_point cycleStart;
    bool on;
    
};

#endif
