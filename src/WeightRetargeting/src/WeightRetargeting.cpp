#include <algorithm>
#include <unordered_map>
#include <mutex>
#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IEncodersTimed.h>

#include <thrift/WeightRetargetingService.h>
#include <thrift/WearableActuatorCommand.h>

#include "WeightRetargetingLogComponent.h"

#include "CommandGeneratorFactory.h"

#define WEIGHT_RETARGETING_MAX_INTENSITY 127

class WeightRetargetingModule : public yarp::os::RFModule, WeightRetargetingService
{
public:

    struct ActuatorGroupInfo
    {
        std::vector<int> jointIndexes;
        double minThreshold;
        double maxThreshold;
        double offset;
        std::vector<std::string> actuators;
    };

    enum class RetargetedValue
    {
        JointTorque,
        MotorCurrent,
        Invalid
    };

    static RetargetedValue retargetedValuefromString(const std::string& name)
    {
        if(name=="joint_torque")
            return RetargetedValue::JointTorque;
        if(name=="motor_current")
            return RetargetedValue::MotorCurrent;
        
        return RetargetedValue::Invalid;
    }

    const std::string LOG_PREFIX = "HapticModule"; 

    const std::string IFEEL_SUIT_ACTUATOR_PREFIX = "iFeelSuit::haptic::Node#";
    
    // Number of configuration parameters defining an actuator group
    const int CONFIG_GROUP_SIZE = 5;

    double period = 0.02; //Default 50Hz

    std::mutex mutex;

    yarp::dev::PolyDriver remappedControlBoard;

    // RetargetedValue
    RetargetedValue retargetedValue;
    yarp::dev::ITorqueControl* iTorqueControl{ nullptr };
    yarp::dev::ICurrentControl* iCurrentControl{ nullptr };
    
    // Velocity check parameters
    bool useVelocities = false;
    yarp::dev::IEncodersTimed* iEncodersTimed{ nullptr };
    std::vector<double> velocities;
    double maxJointVelocity = 0.35;

    std::vector<std::string> remoteControlBoards;
    std::vector<std::string> jointNames;
    std::unordered_map<std::string,ActuatorGroupInfo> actuatorGroupMap; 

    // Data acquisition variables
    std::vector<double> interfaceValues;
    const std::chrono::milliseconds ACQUISITION_TIMEOUT = std::chrono::milliseconds(5000);
    std::chrono::time_point<std::chrono::system_clock> lastAcquisition;

    // Haptic command
    yarp::os::BufferedPort<wearable::msg::WearableActuatorCommand> actuatorCommandPort;
    double minIntensity = 0.0;

    // RPC
    yarp::os::Port rpcPort;

    double getPeriod() override
    {
        return period; //50Hz
    }

    /**
     * @brief Get the square norms of the retargeted interface of an actuator group
     * 
     * @param groupInfo the actuators group
     * @return double the norm of the read measurements
     */
    double getNorm(const ActuatorGroupInfo& groupInfo)
    {
        double sum = 0;
        for(const int &index : groupInfo.jointIndexes)
        {
            sum += interfaceValues[index] * interfaceValues[index];
        }
        return std::sqrt(sum);
    }

    double computeActuationIntensity(const double measuredValue, const double minThreshold, const double maxThreshold)
    {
        double actuationIntensity = 0.0;
        double normalizedValue = (measuredValue - minThreshold) / (maxThreshold - minThreshold);
        if(normalizedValue>0)
        {
            if(normalizedValue>1.0) normalizedValue = 1.0;

            //TODO check if it's better to use steps
            actuationIntensity = (int)(normalizedValue*WEIGHT_RETARGETING_MAX_INTENSITY);
        }
        return actuationIntensity;
    }

    /**
     * @brief Check the max velocity constraint for an actuator group
     * 
     * @param groupInfo the group to check
     * @return true if none of the related joints' velocity is above threshold
     * @return false otherwise
     */
    bool checkGroupVelocity(const ActuatorGroupInfo& groupInfo)
    {
        for(const int &index : groupInfo.jointIndexes)
        {
            if(velocities[index]>maxJointVelocity)
            {
                return false;
            }
        }

        return true;
    }

    /**
     * @brief Computes the actuation command value of a group
     * 
     * @param groupInfo the actuators group
     * @return double the value of the actuation command
     */
    double computeActuationIntensity(const ActuatorGroupInfo& groupInfo)
    {
        //check group velocity
        if(useVelocities && !checkGroupVelocity(groupInfo))
        {
            return 0;
        }
        
        // compute the norm
        double norm = getNorm(groupInfo);
        
        // remove offset
        norm = norm+groupInfo.offset;

        return computeActuationIntensity(norm, groupInfo.minThreshold, groupInfo.maxThreshold);
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
            if(groupName=="all")
            {
                yCError(WEIGHT_RETARGETING_LOG_COMPONENT) << "All is a reserved name for actuator groups";
                return false;
            }
            else if(actuatorGroupMap.find(groupName)!=actuatorGroupMap.end())
            {
                yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Multiple definition of actuator group"<<groupName;
                return false;
            }

            //get axis names
            std::vector<std::string> jointAxes;
            yarp::os::Value& jointsList = groupInfoBottle->get(1);
            if(jointsList.isList())
            {
                yarp::os::Bottle* jointsListBottle = jointsList.asList();
                for(int i=0;i<jointsListBottle->size(); i++)
                {
                    jointAxes.push_back(jointsListBottle->get(i).asString());
                }
            }
            else
            {
                jointAxes.push_back(jointsList.asString());
            }

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

            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Added actuator group: name"<<groupName//TODO axes  <<"| Joint axis"<<jointAxes
                                                      <<"| Min threshold"<< groupInfo.minThreshold << "| Max threshold"<< groupInfo.maxThreshold;

            //add joint axis name to the list
            for(std::string& axisName : jointAxes)
            {
                auto it = std::find(jointNames.begin(), jointNames.end(), axisName);
                if(it==jointNames.end())
                {
                    groupInfo.jointIndexes.push_back(jointNames.size());
                    jointNames.push_back(axisName);
                }
                else
                {
                    groupInfo.jointIndexes.push_back(it - jointNames.begin());
                }
            }
            
            // add group info to the map
            groupInfo.offset = 0.0;
            actuatorGroupMap[groupName] = groupInfo;
        }
        
        return true;
    }

    /**
     * @brief Generates the actuation commands for all of the configured groups
     * 
     */
    void generateGroupsActuation()
    {
        for(auto const & pair : actuatorGroupMap)
        {
            const ActuatorGroupInfo& actuatorGroupInfo = pair.second;

            double actuationIntensity = computeActuationIntensity(actuatorGroupInfo);
            if(actuationIntensity>minIntensity)
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
    }

    bool updateModule() override
    {
        std::lock_guard<std::mutex> guard(mutex);
        auto currentTime = std::chrono::system_clock::now();
        
        // get the data 
        std::vector<double> buffer;
        bool acquisitionResult = false;
        buffer.resize(jointNames.size());
        switch (retargetedValue)
        {
        case RetargetedValue::JointTorque : acquisitionResult = iTorqueControl->getTorques(buffer.data()); break;
        case RetargetedValue::MotorCurrent : acquisitionResult = iCurrentControl->getCurrents(buffer.data()); break;
        default: acquisitionResult = false; break;
        } 

        // check if acquisition was successful
        if(acquisitionResult)
        {
            // update internal data only if acquisition is successful
            for(int i=0;i<jointNames.size();i++)
            {
                interfaceValues[i] = buffer[i];
            }

            lastAcquisition = currentTime;

            // get the velocities
            if(useVelocities)
            {
                if(iEncodersTimed->getEncoderSpeeds(buffer.data()))
                {
                    for(int i=0;i<jointNames.size();i++)
                    {
                        velocities[i] = buffer[i];
                    }
                }
            }

            // generate the actuation commands
            generateGroupsActuation();

        } else if(currentTime-lastAcquisition > ACQUISITION_TIMEOUT)
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Data acquisition timeout has expired!";
            return false;
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

        // read use_velocity param
        if(!rf.check("use_velocity"))
        {
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter use_velocity, using default value" << useVelocities;
        } else 
        {
            useVelocities = rf.find("use_velocity").asBool();
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter use_velocity:" << useVelocities;
        }

        if(useVelocities)
        {
            if(!rf.check("max_velocity"))
            {
                yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter max_velocity, using default value" << maxJointVelocity;
            } else 
            {
                maxJointVelocity = rf.find("max_velocity").asFloat64();
                yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter max_velocity:" << maxJointVelocity;
            }
        }   

        // read retargeted_value param
        if(!rf.check("retargeted_value"))
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter: retargeted_value";
            return false;
        } else 
        {
            retargetedValue = retargetedValuefromString(rf.find("retargeted_value").asString());
            if(retargetedValue==RetargetedValue::Invalid)
            {
                yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Invalid retargeted_value value:"<< rf.find("retargeted_value").asString();
                return false;
            }
        }

        // read min_actuation param
        if(!rf.check("min_intensity"))
        {
            yCDebug(WEIGHT_RETARGETING_LOG_COMPONENT) << "Missing parameter min_intensity, using default value" << minIntensity;
        } else 
        {
            minIntensity = rf.find("min_intensity").asFloat64();
            yCDebug(WEIGHT_RETARGETING_LOG_COMPONENT) << "Found parameter min_intensity:" << minIntensity;
        }

        // read remote_boards param 
        yarp::os::Bottle* remoteBoardsBottle = rf.find("remote_boards").asList();
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
        yarp::os::Property& remoteControlBoardsOpts = propRemapper.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
        remoteControlBoardsOpts.put("writeStrict", "off");

        // open the remapped control board
        result = remappedControlBoard.open(propRemapper);
        if(!result)
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Unable to open the ControlBoardRemapper";
            return result;
        }

        // get the interface related to the corresponding retargeted value
        switch(retargetedValue)
        {
        case RetargetedValue::JointTorque: result = remappedControlBoard.view(iTorqueControl); break;
        case RetargetedValue::MotorCurrent: result = remappedControlBoard.view(iCurrentControl); break;
        default : result = false;
        }

        if(!result)
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Unable to get torque control interface";
            return result;
        }

        // get the interface for the velocity if needed
        if(useVelocities && !remappedControlBoard.view(iEncodersTimed))
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Unable to get encodersTimed interface";
            return false;
        }

        interfaceValues.resize(jointNames.size());
        velocities.resize(jointNames.size());

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

        lastAcquisition = std::chrono::system_clock::now();

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
        std::lock_guard<std::mutex> guard(mutex);
        if(actuatorGroupMap.find(actuatorGroup)==actuatorGroupMap.end())
            return false;

        actuatorGroupMap[actuatorGroup].maxThreshold = value;
        return true;
    }

    bool setMinThreshold(const std::string& actuatorGroup, const double value) override
    {
        std::lock_guard<std::mutex> guard(mutex);
        if(actuatorGroupMap.find(actuatorGroup)==actuatorGroupMap.end())
            return false;

        actuatorGroupMap[actuatorGroup].minThreshold = value;
        return true;
    }

    bool setThresholds(const std::string& actuatorGroup, const double minThreshold, const double maxThreshold) override
    {
        std::lock_guard<std::mutex> guard(mutex);
        if(actuatorGroupMap.find(actuatorGroup)==actuatorGroupMap.end())
            return false;

        actuatorGroupMap[actuatorGroup].minThreshold = minThreshold;
        actuatorGroupMap[actuatorGroup].maxThreshold = maxThreshold;
        return true;
    }

    void removeSingleOffset(const std::string& actuatorGroup)
    {
        ActuatorGroupInfo& groupInfo = actuatorGroupMap[actuatorGroup];

        actuatorGroupMap[actuatorGroup].offset = groupInfo.minThreshold - getNorm(groupInfo);
    }

    bool removeOffset(const std::string& actuatorGroup) override
    {
        std::lock_guard<std::mutex> guard(mutex);
        if(actuatorGroup=="all")
        {
            for(auto const & pair : actuatorGroupMap)
                removeSingleOffset(pair.first);
        }
        else
        {
            if(actuatorGroupMap.find(actuatorGroup)==actuatorGroupMap.end())
                return false;
            
            removeSingleOffset(actuatorGroup);
        }

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

    if (module.runModule(rf)!=0) {
        yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, module.LOG_PREFIX) << "Module did not start.";
    }

    return 0;
}
