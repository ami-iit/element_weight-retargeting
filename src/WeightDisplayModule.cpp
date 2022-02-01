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
    std::string labelRPCServerPortName;
    std::string labelRPCClientPortName;
    int labelID = 0;
    yarp::os::RpcClient rpcClient;

    double getPeriod() override
    {
        return period; //50Hz
    }

    bool updateModule() override
    {
        //TODO read z-axis force
        double zForce = 0.0;

        //TODO calculate weight
        double weight = 0.0;

        //write to port
        yarp::os::Bottle& weightLabelMessage = outPort.prepare();
        weightLabelMessage.addString(std::to_string(weight));
        outPort.write(false);

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
            period = rf.find("period").asFloat64();
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
            labelRPCServerPortName = std::string();
            yCWarning(WEIGHT_RETARGETING_LOG_COMPONENT) << "Missing parameter rpc_port, the RPC to enable the label will not be used";
        } else 
        {
            labelRPCServerPortName = rf.find("rpc_port").asString();
            labelRPCClientPortName = portPrefix+labelRPCServerPortName;
            yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Found parameter rpc_port:" << labelRPCServerPortName;

            if(!rf.check("label_id"))
            {
                yCWarning(WEIGHT_RETARGETING_LOG_COMPONENT) << "Missing parameter label_id, using default value:" << labelID;
            }
            else
            {
                labelID = rf.find("label_id").asInt32();
                yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Found parameter label_id:" << labelID;
            }
        }

        return true;
    }

    bool configure(yarp::os::ResourceFinder &rf) override
    {
        // read parameters
        if(!loadParams(rf))
        {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Error in parameter retrieval, stopping";
            return false;
        }

        // open output port
        if(!outPort.open(outPortName))
        {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Unable to open output port:"<< outPortName;
            return false;
        }
        else
        {
            yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Opened output port:"<< outPortName;
        }

        // connect to the RPC port
        if(!labelRPCServerPortName.empty())
        {
            if(!rpcClient.open(labelRPCClientPortName))
            {
                yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Unable to open RPC client port:"<< labelRPCClientPortName;
                return false;
            }

            if(!yarp::os::Network::connect(labelRPCClientPortName, labelRPCServerPortName))
            {
                yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Unable to connect to RPC server port:"<< labelRPCServerPortName;
                return false;
            }

            yarp::os::Bottle command;
            yarp::os::Bottle response;
            command.addInt32(labelID);

            yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Sending message to the label RPC:" << command.toString();
            rpcClient.write(command, response);
            yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Response from RPC:" << response.toString();

            // close the RPC port
            rpcClient.close();
            yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Closing RPC client port:" << labelRPCClientPortName;
        }

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
