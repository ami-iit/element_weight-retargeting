#include <algorithm>
#include <unordered_map>
#include <memory>
#include <iomanip>
#include <math.h>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncodersTimed.h>

#include "WeightRetargetingLogComponent.h"

class WeightDisplayModule : public yarp::os::RFModule
{
public:

    const std::string LOG_PREFIX = "DisplayModule"; 

    const double GRAVITY_ACCELERATION = 9.81;
    const int FRACTIONAL_DIGITS = 1; // Number of digits of the weight's fractional part  

    double period = 0.02; //Default 50Hz

    double minWeight = 0.0; // minimum weight to be displayed
    double weightOffset = 0.0; // weight offset due to measurements

    bool useOnlyZ = false;

    // input port
    std::vector<std::string> inputPortNames;
    std::vector<std::unique_ptr<yarp::os::BufferedPort<yarp::sig::Vector>>> inputPorts;

    // output port
    std::string portPrefix = "/WeightDisplay";
    std::string outPortName;
    yarp::os::BufferedPort<yarp::os::Bottle> outPort;

    //use velocity info
    struct VelocityHelper
    {
        bool useVelocity = false;
        std::string robotName;
        double maxVelocity;
        std::vector<std::string> remoteBoards;
        std::vector<std::string> jointAxes;
        yarp::dev::PolyDriver remappedControlBoard;
        std::unordered_map<std::string, std::vector<int>> portToJoints;
        yarp::dev::IEncodersTimed* iEncodersTimed{nullptr};
    };

    VelocityHelper velocityHelper;
    std::vector<double> jointVelBuffer;

    double getPeriod() override
    {
        return period; //50Hz
    }

    bool updateModule() override
    {
        bool getVelocityResult = false;
        if(velocityHelper.useVelocity)
        {
            getVelocityResult = velocityHelper.iEncodersTimed->getEncoderSpeeds(jointVelBuffer.data());
        }

        if(velocityHelper.useVelocity && !getVelocityResult)
        {
            //TODO use timeout
            //skip cycle
            return true;
        }

        // sum forces
        double forces = 0.0;
        for(auto const & port : inputPorts)
        {
            // check on the velocity
            bool readFromPort = true;
            if(velocityHelper.useVelocity)
            {
                auto const& jointsIndices = velocityHelper.portToJoints[port->getName()];  
                for(int i=0; i<jointsIndices.size(); i++)
                {
                    if(jointVelBuffer[jointsIndices[i]]>velocityHelper.maxVelocity)
                    {
                        readFromPort = false;
                    }
                }
            }

            // add the force if the velocity check is passed
            if(readFromPort)
            {
                yarp::sig::Vector* wrench = port->read();
                if(wrench==nullptr)
                {
                    continue;
                }

                if(useOnlyZ)
                {
                    if((*wrench)[2]<0)
                        forces += -(*wrench)[2];
                }
                else //use norm
                {
                    double norm = 0;
                    for(int i = 0; i<3; i++)
                        norm += (*wrench)[i]*(*wrench)[i];
                    norm = sqrt(norm);
                    forces += norm;
                }
                
            }
            
        }

        // calculate weight
        double weight = forces/GRAVITY_ACCELERATION - weightOffset;

        // write to port
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

    bool readVelocityInfoGroup(yarp::os::ResourceFinder &rf)
    {
        yarp::os::Bottle velocityUtilsGroup = rf.findGroup("VELOCITY_UTILS");
        if(velocityUtilsGroup.isNull())
        {
            // use default
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Group VELOCITY_UTILS not found, velocity information will not be used";
            return true; 
        }

        // read use_velocity 
        if(!velocityUtilsGroup.check("use_velocity"))
        {
            // use default
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter use_velocity. Using default value:"<<velocityHelper.useVelocity;
            return true;
        }
        velocityHelper.useVelocity = velocityUtilsGroup.find("use_velocity").asBool();
        yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Parameter use_velocity is:"<<velocityHelper.useVelocity;

        if(!velocityHelper.useVelocity)
        {
            return true;
        }

        // read max_velocity
        if(!velocityUtilsGroup.check("max_velocity"))
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter max_velocity when use_velocity is enabled";
            return false;
        }
        velocityHelper.maxVelocity = velocityUtilsGroup.find("max_velocity").asFloat64();

        // read robot
        if(!velocityUtilsGroup.check("robot"))
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter robot when use_velocity is enabled";
            return false;
        }
        velocityHelper.robotName = velocityUtilsGroup.find("robot").asString();
        if (velocityHelper.robotName[0]!='/')
        {
            velocityHelper.robotName = "/"+velocityHelper.robotName;
        }

        // read remote boards
        if(!velocityUtilsGroup.check("remote_boards"))
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter remote_boards when use_velocity is enabled";
            return false;
        }

        yarp::os::Bottle* remoteBoardsBottle = velocityUtilsGroup.find("remote_boards").asList();
        if(remoteBoardsBottle->size()==0)
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Parameter remote_boards cannot be an empty list!";
            return false;
        }
        for(int i=0;i<remoteBoardsBottle->size();i++)
        {
            std::string remoteBoard = remoteBoardsBottle->get(i).asString();\
            if(remoteBoard[0]!='/') remoteBoard = "/" + remoteBoard; 

            velocityHelper.remoteBoards.push_back(remoteBoard);
        }

        // read joints_info
        if(!velocityUtilsGroup.check("joints_info"))
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter remote_boards when joints_info is enabled";
            return false;
        }
        yarp::os::Bottle* jointsInfoBottle = velocityUtilsGroup.find("joints_info").asList();
        if(jointsInfoBottle->size()==0)
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Parameter joints_info cannot be an empty list!";
            return false;
        }
        for(int i=0;i<jointsInfoBottle->size();i++)
        {
            yarp::os::Bottle* infoBottle = jointsInfoBottle->get(i).asList(); 
            if(infoBottle->size()<2)
            {
                yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Bad joints_info format!";
                return false;
            }
            // read port name
            std::string portName = portPrefix+"/"+infoBottle->get(0).asString();


            // read joint axes
            std::vector<int> jointsIndices = {};
            for(int j=1; j<infoBottle->size(); j++)
            {
                jointsIndices.push_back(velocityHelper.jointAxes.size());
                velocityHelper.jointAxes.push_back(infoBottle->get(j).asString());
            }

            velocityHelper.portToJoints[portName] = jointsIndices;
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

        // read weight_offset
        if(!rf.check("weight_offset"))
        {
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter weight_offset from configuration";
        }
        weightOffset = rf.check("weight_offset", yarp::os::Value(0.0)).asFloat64();
        yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Using parameter weight_offset:"<<weightOffset;

        // read use_z_only
        if(!rf.check("use_z_only"))
        {
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter use_z_only from configuration";
        }
        useOnlyZ = rf.check("use_z_only", yarp::os::Value(false)).asBool();
        yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Using parameter use_z_only:"<<useOnlyZ;

        // read velocity info
        return readVelocityInfoGroup(rf);
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

        // manage use velocity
        if(velocityHelper.useVelocity)
        {
            // set the size of the data buffer
            jointVelBuffer.resize(velocityHelper.jointAxes.size());

            // configure the remapper
            yarp::os::Property propRemapper;
            propRemapper.put("device", "remotecontrolboardremapper");
            // axes names
            propRemapper.addGroup("axesNames");
            yarp::os::Bottle& axesNamesBottle = propRemapper.findGroup("axesNames").addList();
            for(std::string& s : velocityHelper.jointAxes) axesNamesBottle.addString(s);
            // remote control boards names
            propRemapper.addGroup("remoteControlBoards");
            yarp::os::Bottle& remoteControlBoardsNamesBottle = propRemapper.findGroup("remoteControlBoards").addList();
            for(std::string& s : velocityHelper.remoteBoards) remoteControlBoardsNamesBottle.addString(velocityHelper.robotName+s);
            // localPortPrefix
            propRemapper.put("localPortPrefix", "/WeightDisplay/input");
            yarp::os::Property& remoteControlBoardsOpts = propRemapper.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
            remoteControlBoardsOpts.put("writeStrict", "off");

            if(!velocityHelper.remappedControlBoard.open(propRemapper))
            {
                yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Cannot open remoteControlBoardRemapper!";
                return false;
            }

            if(!velocityHelper.remappedControlBoard.view(velocityHelper.iEncodersTimed))
            {
                yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Cannot view iEncodersTimed!";
                return false;
            }
        }

        yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT,  LOG_PREFIX) << "Module started successfully!";

        return true;
    }

    bool close() override
    {
        // close input ports
        for(auto & port : inputPorts)
            port->close();

        // close the control board remapper
        if(velocityHelper.useVelocity)
        {
            velocityHelper.remappedControlBoard.close();
        }

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
