#include "TimePatterns/PulseFeedback.h"

using namespace WeightRetargeting;

//TODO
#define PULSE_DURATION_MS 600

patterns::PulseFeedback::PulseFeedback():TimePattern()
{
    level = -1;
    on = false;
}

void patterns::PulseFeedback::makeFrequencies(const int levels, const double maxFrequency)
{
    periods.clear();
    levelThresholds.clear();

    const double frequencyStep = maxFrequency/(levels-1);
    double frequency = 0.0;

    if (levels == 1)
    {
        levelThresholds.push_back(0.5);
        periods.push_back(1000/maxFrequency);
    }
    else
    {
        for(int i = 1; i < levels ; i++)
    {
        frequency += frequencyStep;
            levelThresholds.push_back(i/(double)levels);
        periods.push_back(1000/frequency);
    }
}

}

void patterns::PulseFeedback::makeFrequencies(std::vector<double> thresholds, std::vector<double> frequencies)
{
    //TODO frequencies now are periods in seconds

    periods.clear();
    levelThresholds.clear();
    
    this->levelThresholds = thresholds;
    for(auto const f : frequencies)
    {
        this->periods.push_back(1000*f);
    }
}

void patterns::PulseFeedback::setCustomActuation(const double customActuation)
{
    this->customActuation = customActuation;
}

void patterns::PulseFeedback::update(double value)
{
    TimePattern::update(value);

    //TODO get level index
    int currentLevel = -1;
    for(int i = levelThresholds.size()-1; i>=0 && currentLevel==-1; i--)
    {
        if(value>=levelThresholds[i])
        {
            currentLevel = i;
        }
    }

    // Start new cycle if we passed to a new level or if the period is finished

    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(lastTimestamp-cycleStart).count();
    if(currentLevel!=-1 && (currentLevel!=level || elapsed > periods[currentLevel]))
    {
        cycleStart = lastTimestamp;
        elapsed = 0;
    }

    on = (elapsed<PULSE_DURATION_MS);

    level = currentLevel;
}

double patterns::PulseFeedback::getCommand()
{
    if(level==-1 || !on)
    {
        return 0.;
    }

    
    if(customActuation<=0.)
    {
        // propagate input value
        return lastValue;
    }
    else
    {
        return customActuation;
    }
    
}
