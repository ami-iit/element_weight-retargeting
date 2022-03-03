#include <algorithm>
#include <unordered_map>
#include <memory>
#include <iomanip>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include "WeightRetargetingLogComponent.h"

class WeightDisplayModule : public yarp::os::RFModule
{
public:

    const std::string LOG_PREFIX = "DisplayModule"; 

    const double GRAVITY_ACCELERATION = 9.81;
    const int FRACTIONAL_DIGITS = 3; // Number of digits of the weight's fractional part  

    double period = 0.02; //Default 50Hz

    double minWeight = 0.0; // minimum weight to be displayed

    // input port
    std::vector<std::string> inputPortNames;
    std::vector<std::unique_ptr<yarp::os::BufferedPort<yarp::sig::Vector>>> inputPorts;

    // output port
    std::string portPrefix = "/WeightDisplay";
    std::string outPortName;
    yarp::os::BufferedPort<yarp::os::Bottle> outPort;

    double getPeriod() override
    {
        return period; //50Hz
    }

    bool updateModule() override
    {
        // sum z-axis forces
        double zForce = 0.0;
        for(auto const & port : inputPorts)
        {
            yarp::sig::Vector* wrench = port->read(false);
            if(wrench!=nullptr && (*wrench)[2]<0)
                zForce += -(*wrench)[2];
        }

        // calculate weight
        double weight = zForce/GRAVITY_ACCELERATION;

        //write to port
        if(weight>=minWeight)
        {
            // use stringstream to fix number of fractional digits
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(FRACTIONAL_DIGITS)<<weight;

            yarp::os::Bottle& weightLabelMessage = outPort.prepare();
            weightLabelMessage.clear();
            weightLabelMessage.addString(stream.str());
            outPort.write(false);
        }

        return true;
    }

    bool loadParams(yarp::os::ResourceFinder &rf)
    {
        // read period param
        if(!rf.check("period"))
        {
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter period, using default value" << period;
        } else 
        {
            period = rf.find("period").asFloat64();
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter period:" << period;
        }

        // read port_prefix param
        if(!rf.check("port_prefix"))
        {
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter port_prefix, using default value:" << portPrefix;
        } else 
        {
            portPrefix = rf.find("port_prefix").asString();
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter port_prefix:" << portPrefix;
        }

        // read input port names
        yarp::os::Bottle* inputPortNamesBottle = rf.find("input_port_names").asList();
        if(inputPortNamesBottle == nullptr)
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter input_port_names!";
            return false;
        }
        else if(inputPortNamesBottle->size()==0)
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Parameter input_port_names is empty!";
            return false;
        }
        
        for(int i=0; i<inputPortNamesBottle->size(); i++)
        {
            std::string portName = inputPortNamesBottle->get(i).asString();
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found input port name:"<<portName;
            inputPortNames.push_back(portPrefix+"/"+portName+":i");
        }

        // read min_weight
        if(!rf.check("min_weight"))
        {
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter min_weight, using default value:"<<minWeight;
        } else
        {
            minWeight = rf.find("min_weight").asFloat64();
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter min_weight:" << minWeight;
        }

        return true;
    }

    bool configure(yarp::os::ResourceFinder &rf) override
    {
        // read parameters
        if(!loadParams(rf))
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Error in parameter retrieval, stopping";
            return false;
        }

        // open input ports
        int inputPortIdx = 0;
        inputPorts.reserve(inputPortNames.size());
        for(const std::string& portName : inputPortNames)
        {
            inputPorts.push_back(std::make_unique<yarp::os::BufferedPort<yarp::sig::Vector>>());
            if(!inputPorts[inputPortIdx++]->open(portName))
            {
                yCError(WEIGHT_RETARGETING_LOG_COMPONENT)<<"Unable to open input port:"<< portName;
                return false;
            }
        }

        // open output port
        outPortName = portPrefix+"/out:o";
        if(!outPort.open(outPortName))
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Unable to open output port:"<< outPortName;
            return false;
        }

        yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT,  LOG_PREFIX) << "Module started successfully!";

        return true;
    }

    bool close() override
    {
        // close input ports
        for(auto & port : inputPorts)
            port->close();

        // close output port
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

    yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, module.LOG_PREFIX) << "Configuring and starting module.";

    if (module.runModule(rf)!=0) {
        yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, module.LOG_PREFIX) << "Module did not start.";
    }

    return 0;
}
