#include <algorithm>
#include <unordered_map>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>

#include "WeightRetargetingLogComponent.h"

class WeightDisplayModule : public yarp::os::RFModule
{
public:

    double period = 0.02; //Default 50Hz

    // output port
    std::string portPrefix = "/WeightDisplay";
    std::string outPortName;
    yarp::os::BufferedPort<yarp::os::Bottle> outPort;

    // RPC
    std::string labelRPCPortName;
    yarp::os::RpcClient rpcClient;

    double getPeriod() override
    {
        return period; //50Hz
    }

    bool updateModule() override
    {
        //TODO
        return true;
    }

    bool loadParams(yarp::os::ResourceFinder &rf)
    {
        // read period param
        if(!rf.check("period"))
        {
            yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Missing parameter period, using default value" << period;
        } else 
        {
            period = rf.find("period").asDouble();
            yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Found parameter period:" << period;
        }

        // read port_prefix param
        if(!rf.check("port_prefix"))
        {
            yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Missing parameter port_prefix, using default value:" << portPrefix;
        } else 
        {
            portPrefix = rf.find("port_prefix").asString();
            yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Found parameter port_prefix:" << portPrefix;
        }

        outPortName = portPrefix+"/out";


        // read rpc_port param
        if(!rf.check("rpc_port"))
        {
            labelRPCPortName = std::string("");
            yCWarning(WEIGHT_RETARGETING_LOG_COMPONENT) << "Missing parameter rpc_port, the RPC to enable the label will not be used";
        } else 
        {
            labelRPCPortName = rf.find("rpc_port").asString();
            yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Found parameter rpc_port:" << labelRPCPortName;
        }

        return true;
    }

    bool configure(yarp::os::ResourceFinder &rf) override
    {
        if(!loadParams(rf))
        {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Error in parameter retrieval, stopping";
            return false;
        }

        //TODO open ports

        return true;
    }

    bool interruptModule() override
    {
        //TODO
        return true;
    }

    bool close() override
    {
        outPort.close();
        return true;
    }

};

int main(int argc, char * argv[])
{
    // Initialize yarp network
    yarp::os::Network::init();

    // create your module
    WeightDisplayModule module;

    // prepare and configure the resource finder
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("WeightRetargeting");
    rf.configure(argc, argv);

    yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Configuring and starting module.";

    if (!module.runModule(rf)) {
        yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Module did not start.";
    }

    return 0;
}
