#include "TimePatterns/PulseFeedback.h"

using namespace WeightRetargeting;

//TODO
#define PULSE_DURATION_MS 250

patterns::PulseFeedback::PulseFeedback():TimePattern()
{
    level = -1;
    on = false;
}

void patterns::PulseFeedback::makeFrequencies(const int levels, const double maxFrequency)
{
    periods.clear();
    levelThresholds.clear();

    const double frequencyStep = maxFrequency/levels;
    double frequency = 0.0;

    for(int i = levels; i>1 ; i--)
    {
        frequency += frequencyStep;
        levelThresholds.push_back(1.0/i);
        periods.push_back(1000/frequency);
    }
}

void patterns::PulseFeedback::makeFrequencies(std::vector<double> thresholds, std::vector<double> frequencies)
{
    //TODO

    periods.clear();
    levelThresholds.clear();
    
    this->levelThresholds = thresholds;
    for(auto const f : frequencies)
    {
        this->periods.push_back(1000/f);        
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
    while(currentLevel<levelThresholds.size() && value<levelThresholds[currentLevel+1])
    {
        currentLevel++;
    }

    if(currentLevel!=level)
    {
        cycleStart = lastTimestamp;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(lastTimestamp-cycleStart).count(); 
    if(elapsed>periods[currentLevel])
    {
        cycleStart = lastTimestamp;
        elapsed -= elapsed;
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
