#include "TimePatterns/PulseFeedback.h"

using namespace WeightRetargeting;

void patterns::PulseFeedback::makeFrequencies(const int levels, const double maxFrequency)
{
    //TODO distribute the frequencies from 0 to maxFrequency in level   
}

void patterns::PulseFeedback::makeFrequencies(std::vector<double> thresholds, std::vector<double> frequencies)
{
    //TODO
}

void patterns::PulseFeedback::update(double value)
{
    //TODO
}

double patterns::PulseFeedback::getCommand()
{
    //TODO
    return 0.;
}
