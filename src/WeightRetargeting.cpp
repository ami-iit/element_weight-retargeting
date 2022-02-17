#include <algorithm>
#include <unordered_map>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ITorqueControl.h>

#include <thrift/WeightRetargetingService.h>
#include <thrift/WearableActuatorCommand.h>

#include "WeightRetargetingLogComponent.h"

#define WEIGHT_RETARGETING_MAX_INTENSITY 127

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

    const std::string LOG_PREFIX = "HapticModule"; 

    const std::string IFEEL_SUIT_ACTUATOR_PREFIX = "iFeelSuit::haptic::Node#";
    
    // Number of configuration parameters defining an actuator group
    const int CONFIG_GROUP_SIZE = 5;

    double period = 0.02; //Default 50Hz

    yarp::dev::PolyDriver remappedControlBoard;
    yarp::dev::ITorqueControl* iTorqueControl{ nullptr };

    std::vector<std::string> remoteControlBoards;
    std::vector<std::string> jointNames;
    std::unordered_map<std::string,ActuatorGroupInfo> actuatorGroupMap; 

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

    /**
     * @brief Retrieve data related to actuators groups from configuration
     * 
     * @param rf the ResourceFinder instance
     * @return true if the reading was successful
     * @return false otherwise
     */
    bool readActuatorsGroups(yarp::os::ResourceFinder &rf)
    {

        yarp::os::Bottle* actuatorGroupsBottle = rf.find("actuator_groups").asList();
        if(actuatorGroupsBottle==nullptr)
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter: actuator_groups";
            return false;
        }

        yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter: actuator_groups";
        for(int i=0; i<actuatorGroupsBottle->size(); i++)
        {
            ActuatorGroupInfo groupInfo;
            yarp::os::Bottle* groupInfoBottle = actuatorGroupsBottle->get(i).asList();

            if(groupInfoBottle->size()!=CONFIG_GROUP_SIZE)
            {
                yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "The number of configuration parameter for group"<<i<<"is incorrect (must be"<<CONFIG_GROUP_SIZE<<")";
                return false;
            }

            // get actuator group name
            std::string groupName = groupInfoBottle->get(0).asString();
            if(actuatorGroupMap.find(groupName)!=actuatorGroupMap.end())
            {
                yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Multiple definition of actuator group"<<groupName;
                return false;
            }

            //get axis name
            std::string axisName = groupInfoBottle->get(1).asString();

            //get min threshold
            groupInfo.minThreshold = groupInfoBottle->get(2).asFloat64();

            //get max threshold
            groupInfo.maxThreshold = groupInfoBottle->get(3).asFloat64();

            //get list of actuators
            yarp::os::Bottle* actuatorListBottle = groupInfoBottle->get(4).asList();
            if(actuatorListBottle->size()==0)
            {
                yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "The actuators list of"<<groupName<<"is empty!";
                return false;
            }

            for(int j = 0; j<actuatorListBottle->size(); j++) 
                groupInfo.actuators.push_back(actuatorListBottle->get(j).asString());

            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Added actuator group: name"<<groupName <<"| Joint axis"<<axisName
                                                      <<"| Min threshold"<< groupInfo.minThreshold << "| Max threshold"<< groupInfo.maxThreshold;

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
            actuatorGroupMap[groupName] = groupInfo;
        }
        
        return true;
    }

    bool updateModule() override
    {
        iTorqueControl->getTorques(jointTorques.data());

        for(auto const & pair : actuatorGroupMap)
        {
            const ActuatorGroupInfo& actuatorGroupInfo = pair.second;

            double actuationIntensity = computeActuationIntensity(jointTorques[actuatorGroupInfo.jointIdx], actuatorGroupInfo.minThreshold, actuatorGroupInfo.maxThreshold);

            if(actuationIntensity>0)
            {
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
                    actuatorCommandPort.write(true);
                }
            }

        }

        return true;
    }

    bool configure(yarp::os::ResourceFinder &rf) override
    {
        bool result = true;

        // read robot name
        std::string robotName = rf.find("robot").asString();
        if(robotName.empty())
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter: robot";
            return false;
        } else if (robotName[0]!='/')
        {
            robotName = "/"+robotName;
        }

        // read period param
        if(!rf.check("period"))
        {
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter period, using default value" << period;
        } else 
        {
            period = rf.find("period").asFloat64();
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter period:" << period;
        }

        yarp::os::Bottle* remoteBoardsBottle = rf.find("remote_boards").asList();
        
        // remote control boards 
        if(remoteBoardsBottle==nullptr)
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter: remote_boards";
            return false;
        }

        yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter remote_boards";
        for(int i=0;i<remoteBoardsBottle->size();i++)
        {
            std::string remoteBoard = remoteBoardsBottle->get(i).asString();

            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Added remote control board:" << remoteBoard;

            if(remoteBoard[0]!='/') remoteBoard = "/"+remoteBoard;
            remoteControlBoards.push_back(remoteBoard);
        } 
        
        // Read information about the actuator groups
        if(!readActuatorsGroups(rf))
            return false;

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
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Unable to open the ControlBoardRemapper";
            return result;
        }

        result = remappedControlBoard.view(iTorqueControl);

        if(!result)
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Unable to get torque control interface";
            return result;
        }

        int axes = 0;
        iTorqueControl->getAxes(&axes);
        if(axes!=jointNames.size())
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Number of iTorqueControl axes is different than the configured ones";
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
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Failed to open" << actuatorCommandPort.getName();
            return false;
        }

        // Initialize RPC

        this->yarp().attachAsServer(rpcPort);
        std::string rpcPortName = "/WeightRetargeting/rpc:i"; //TODO from config?
        // open the RPC port
        if(!rpcPort.open(rpcPortName))
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Failed to open" << actuatorCommandPort.getName();
            return false;
        }
        // attach the port
        if (!this->yarp().attachAsServer(rpcPort)) {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Failed to attach" << rpcPortName << "to the RPC service";
            return false;
        }

        yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT,  LOG_PREFIX) << "Module started successfully!";

        return true;
    }

    bool close() override
    {
        actuatorCommandPort.close();
        return true;
    }

    bool setMaxThreshold(const std::string& actuatorGroup, const double value) override
    {
        if(actuatorGroupMap.find(actuatorGroup)==actuatorGroupMap.end())
            return false;

        actuatorGroupMap[actuatorGroup].maxThreshold = value;
        return true;
    }

    bool setMinThreshold(const std::string& actuatorGroup, const double value) override
    {
        if(actuatorGroupMap.find(actuatorGroup)==actuatorGroupMap.end())
            return false;

        actuatorGroupMap[actuatorGroup].minThreshold = value;
        return true;
    }

    bool setThresholds(const std::string& actuatorGroup, const double minThreshold, const double maxThreshold) override
    {
        if(actuatorGroupMap.find(actuatorGroup)==actuatorGroupMap.end())
            return false;

        actuatorGroupMap[actuatorGroup].minThreshold = minThreshold;
        actuatorGroupMap[actuatorGroup].maxThreshold = maxThreshold;
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
    rf.setDefaultContext("WeightRetargeting");
    rf.configure(argc, argv);

    yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, module.LOG_PREFIX) << "Configuring and starting module.";

    if (!module.runModule(rf)) {
        yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, module.LOG_PREFIX) << "Module did not start.";
    }

    return 0;
}
