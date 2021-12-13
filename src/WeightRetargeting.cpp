#include <iostream>

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>

#include <unordered_map>

class WeightRetargetingModule : public yarp::os::RFModule
{
private:

public:
    double getPeriod() override
    {
        return 0.02; //50Hz
    }

    bool updateModule() override
    {
        //TODO
        return true;
    }

    bool configure(yarp::os::ResourceFinder &rf) override
    {
        //TODO
        return true;
    }

    bool interruptModule() override
    {
        //TODO
        return true;
    }

    bool close() override
    {
        //TODO
        return true;
    }

};

int main(int argc, char * argv[])
{
    // Initialize yarp network
    yarp::os::Network::init();

    // create your module
    WeightRetargetingModule module;

    // prepare and configure the resource finder
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    yInfo() << "Configuring and starting module.\n";

    if (!module.runModule(rf)) {
        yError() << "Error module did not start\n";
    }

    return 0;
}
