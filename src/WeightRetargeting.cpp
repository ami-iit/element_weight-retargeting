#include <algorithm>
#include <unordered_map>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ITorqueControl.h>

#include <thrift/WeightRetargetingService.h>
#include <thrift/WearableActuatorCommand.h>

#define WEIGHT_RETARGETING_MAX_INTENSITY 127

YARP_LOG_COMPONENT(WEIGHT_RETARGETING_LOG_COMPONENT, "WeightRetargetingModule")

class WeightRetargetingModule : public yarp::os::RFModule, WeightRetargetingService
{
public:

    struct ActuatorGroupInfo
    {
        int jointIdx;
        double minThreshold;
        double maxThreshold;
        std::vector<std::string> actuators;
    };

    const std::string IFEEL_SUIT_ACTUATOR_PREFIX = "iFeelSuit::haptic::Node#";

    double period = 0.02; //Default 50Hz

    yarp::dev::PolyDriver remappedControlBoard;
    yarp::dev::ITorqueControl* iTorqueControl{ nullptr };

    std::vector<std::string> remoteControlBoards;
    std::vector<std::string> jointNames;
    std::unordered_map<std::string,ActuatorGroupInfo> actuatorGroupInfos; 

    std::vector<double> jointTorques;

    // Haptic command
    yarp::os::BufferedPort<wearable::msg::WearableActuatorCommand> actuatorCommandPort;

    // RPC
    yarp::os::Port rpcPort;

    double getPeriod() override
    {
        return period; //50Hz
    }

    double computeActuationIntensity(const double measuredTorque, const double minThreshold, const double maxThreshold)
    {
        double actuationIntensity = 0.0;
        double normalizedTorque = (measuredTorque - minThreshold) / (maxThreshold - minThreshold);
        if(normalizedTorque>0)
        {
            if(normalizedTorque>1.0) normalizedTorque = 1.0;

            //TODO check if it's better to use steps
            actuationIntensity = (int)(normalizedTorque*WEIGHT_RETARGETING_MAX_INTENSITY);
        }
        return actuationIntensity;
    }

    bool updateModule() override
    {
        iTorqueControl->getTorques(jointTorques.data());

        for(auto const & pair : actuatorGroupInfos)
        {
            const ActuatorGroupInfo& actuatorGroupInfo = pair.second;

            double actuationIntensity = computeActuationIntensity(jointTorques[actuatorGroupInfo.jointIdx], actuatorGroupInfo.minThreshold, actuatorGroupInfo.maxThreshold);

            if(actuationIntensity>0)
            {
                yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Sending"<< actuationIntensity << "to group" << pair.first<< "with"<<jointNames[actuatorGroupInfo.jointIdx]<<"torque"<< jointTorques[actuatorGroupInfo.jointIdx];
                //send the haptic command to all the related actuators
                for(const std::string& actuator : actuatorGroupInfo.actuators)
                { 
                    wearable::msg::WearableActuatorCommand& wearableActuatorCommand = actuatorCommandPort.prepare();

                    wearableActuatorCommand.value = actuationIntensity;
                    wearableActuatorCommand.info.name = IFEEL_SUIT_ACTUATOR_PREFIX+actuator;;
                    wearableActuatorCommand.info.type = wearable::msg::ActuatorType::HAPTIC;
                    wearableActuatorCommand.info.status = wearable::msg::ActuatorStatus::OK;
                    wearableActuatorCommand.duration = 0;

                    // Send haptic actuator command
                    // NOTE: Use strict flag true for writing all the commands without dropping any old commands
                    actuatorCommandPort.write(true);
                }
            }
            else
            {
                yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Not actuating the group" << pair.first << ","<<jointNames[actuatorGroupInfo.jointIdx]<<"torque is"<< jointTorques[actuatorGroupInfo.jointIdx];
            }

        }

        return true;
    }

    bool configure(yarp::os::ResourceFinder &rf) override
    {
        bool result = true;

        std::string robotName = rf.find("robot").asString();
        if(robotName.empty())
        {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Missing parameter: robot";
            return false;
        } else if (robotName[0]!='/')
        {
            robotName = "/"+robotName;
        }

        if(!rf.check("period"))
        {
            yCDebug(WEIGHT_RETARGETING_LOG_COMPONENT) << "Missing parameter period, using default value" << period;
        } else 
        {
            period = rf.find("period").asDouble();
            yCDebug(WEIGHT_RETARGETING_LOG_COMPONENT) << "Found parameter period:" << period;
        }

        yarp::os::Bottle* remoteBoardsBottle = rf.find("remote_boards").asList();
        
        // remote control boards 
        if(remoteBoardsBottle==nullptr)
        {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Missing parameter: remote_boards";
            return false;
        }

        yCDebug(WEIGHT_RETARGETING_LOG_COMPONENT) << "remote_boards OK";
        for(int i=0;i<remoteBoardsBottle->size();i++)
        {
            std::string remoteBoard = remoteBoardsBottle->get(i).asString();

            yCDebug(WEIGHT_RETARGETING_LOG_COMPONENT) << "Added remote control board:" << remoteBoard;

            if(remoteBoard[0]!='/') remoteBoard = "/"+remoteBoard;
            remoteControlBoards.push_back(remoteBoard);
        } 
        
        // Read information about the actuator groups
        yarp::os::Bottle* actuatorGroupsBottle = rf.find("actuator_groups").asList();
        if(actuatorGroupsBottle==nullptr)
        {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Missing parameter: actuator_groups";
            return false;
        }

        yCDebug(WEIGHT_RETARGETING_LOG_COMPONENT) << "actuator_groups OK";
        for(int i=0; i<actuatorGroupsBottle->size(); i++)
        {
            ActuatorGroupInfo groupInfo;
            yarp::os::Bottle* groupInfoBottle = actuatorGroupsBottle->get(i).asList();
            // get actuator group name
            std::string groupName = groupInfoBottle->get(0).asString();
            if(actuatorGroupInfos.find(groupName)!=actuatorGroupInfos.end())
            {
                yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Multiple definition of actuator group"<<groupName;
                return false;
            }
            //get axis name
            std::string axisName = groupInfoBottle->get(1).asString();
            //get min threshold
            groupInfo.minThreshold = groupInfoBottle->get(2).asDouble();
            //get max threshold
            groupInfo.maxThreshold = groupInfoBottle->get(3).asDouble();
            //get list of actuators
            yCDebug(WEIGHT_RETARGETING_LOG_COMPONENT) << "Added actuator group: name"<<groupName <<"| Joint axis"<<axisName
                                                      <<"| Min threshold"<< groupInfo.maxThreshold << "| Max threshold"<< axisName;
            yarp::os::Bottle* actuatorListBottle = groupInfoBottle->get(4).asList();
            std::vector<std::string> actuatorList = {};
            for(int j = 0; j<actuatorListBottle->size(); j++) groupInfo.actuators.push_back(actuatorListBottle->get(j).asString());

            //add joint axis name to the list
            auto it = std::find(jointNames.begin(), jointNames.end(), axisName);
            if(it==jointNames.end())
            {
                groupInfo.jointIdx = jointNames.size();
                jointNames.push_back(axisName);
            }
            else
            {
                groupInfo.jointIdx = it - jointNames.begin();
            }
            
            // add group info to the map
            actuatorGroupInfos[groupName] = groupInfo;
        }

        // configure the remapper
        yarp::os::Property propRemapper;
        propRemapper.put("device", "remotecontrolboardremapper");
        // axes names
        propRemapper.addGroup("axesNames");
        yarp::os::Bottle& axesNamesBottle = propRemapper.findGroup("axesNames").addList();
        for(std::string& s : jointNames) axesNamesBottle.addString(s);
        // remote control boards names
        propRemapper.addGroup("remoteControlBoards");
        yarp::os::Bottle& remoteControlBoardsNamesBottle = propRemapper.findGroup("remoteControlBoards").addList();
        for(std::string& s : remoteControlBoards) remoteControlBoardsNamesBottle.addString(robotName+s);
        // localPortPrefix
        propRemapper.put("localPortPrefix", "/WeightRetargeting/input");

        result = remappedControlBoard.open(propRemapper);

        if(!result)
        {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Unable to open the ControlBoardRemapper";
            return result;
        }

        result = remappedControlBoard.view(iTorqueControl);

        if(!result)
        {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Unable to get torque control interface";
            return result;
        }

        int axes = 0;
        iTorqueControl->getAxes(&axes);
        if(axes!=jointNames.size())
        {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Number of iTorqueControl axes is different than the configured ones";
            result = false;
        }
        else
        {
            jointTorques.reserve(axes);
        }

        std::string wearableActuatorCommandPortName = "/WeightRetargeting/output:o";//TODO config

        // Initialize actuator command port and connect to command input port
        if(!actuatorCommandPort.open(wearableActuatorCommandPortName))
        {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Failed to open" << actuatorCommandPort.getName();
            return false;
        }

        // Initialize RPC

        this->yarp().attachAsServer(rpcPort);
        std::string rpcPortName = "/WeightRetargeting/rpc:i"; //TODO from config?
        // open the RPC port
        if(!rpcPort.open(rpcPortName))
        {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Failed to open" << actuatorCommandPort.getName();
            return false;
        }
        // attach the port
        if (!this->yarp().attachAsServer(rpcPort)) {
            yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Failed to attach" << rpcPortName << "to the RPC service";
            return false;
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
        actuatorCommandPort.close();
        return true;
    }

    bool setMaxThreshold(const std::string& axis, const double value) override
    {
        //TODO
        return false;
    }

    bool setMinThreshold(const std::string& axis, const double value) override
    {
        //TODO
        return false;
    }

    bool setThresholds(const std::string& axis, const double minThreshold, const double maxThreshold) override
    {
        //TODO
        return false;
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
    rf.setDefaultContext("WeightRetargeting");
    rf.configure(argc, argv);

    yCInfo(WEIGHT_RETARGETING_LOG_COMPONENT) << "Configuring and starting module.";

    if (!module.runModule(rf)) {
        yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "Module did not start.";
    }

    return 0;
}
