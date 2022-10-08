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

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <thrift/WeightRetargetingService.h>
#include <thrift/WearableActuatorCommand.h>

#include "filters/SecondOrderLowPass.h"

#include "WeightRetargetingLogComponent.h"

#include "ActuatorsGroupFactory.h"

class WeightRetargetingModule : public yarp::os::RFModule, WeightRetargetingService
{
public:

    struct ActuatorGroupHelper
    {
        double lastCommand = 0;
        int groupIndex;
        std::vector<int> interfaceIndexes;
        double offset;
        WeightRetargeting::ActuatorsGroup& info;

        ActuatorGroupHelper(WeightRetargeting::ActuatorsGroup& info): info(info){};
    };

    WeightRetargeting::ActuatorsGroupFactory actuatorsGroupFactory;

    enum class RetargetedValue
    {
        JointTorque,
        MotorCurrent,
        ForcePort,
        Invalid
    };

    static RetargetedValue retargetedValuefromString(const std::string& name)
    {
        if(name=="joint_torque")
            return RetargetedValue::JointTorque;
        if(name=="motor_current")
            return RetargetedValue::MotorCurrent;
        if(name=="force_port")
            return RetargetedValue::ForcePort;
        
        return RetargetedValue::Invalid;
    }

    const std::string LOG_PREFIX = "HapticModule"; 

    const std::string PORT_PREFIX = "/WeightRetargeting";

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
    std::vector<std::unique_ptr<yarp::os::BufferedPort<yarp::sig::Vector>>> forcePorts;
    
    // Velocity check parameters
    bool useVelocities = false;
    yarp::dev::IEncodersTimed* iEncodersTimed{ nullptr };
    std::vector<double> velocities;
    double maxJointVelocity = 0.35;

    std::vector<std::string> remoteControlBoards;
    std::vector<std::string> jointNames;

    std::vector<std::string> actuatorGroupNames;
    std::unordered_map<std::string,ActuatorGroupHelper> actuatorGroupMap;

    // Data acquisition variables
    int interfaceValuesSize;
    std::vector<double> interfaceValues;
    const std::chrono::milliseconds ACQUISITION_TIMEOUT = std::chrono::milliseconds(5000);
    std::chrono::time_point<std::chrono::system_clock> lastAcquisition;

    // Filter
    bool enableFilter = false;
    SecondOrderLowPassFilter lowPassFilter;
    double cutoffFrequency = 1000;

    // Contacts list
    bool useContacts = false;
    yarp::sig::Vector contactsList;
    yarp::os::BufferedPort<yarp::sig::Vector> contactListPort; 

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
    double getNorm(const ActuatorGroupHelper& groupInfo)
    {
        double sum = 0;
        for(const int &index : groupInfo.interfaceIndexes)
        {
            sum += interfaceValues[index] * interfaceValues[index];
        }
        return std::sqrt(sum);
    }

    double computeActuationIntensity(const ActuatorGroupHelper& helper, const double norm)
    {
        double command = norm;
        // generate normalized value
        helper.info.mapFunction->update(command);
        command = helper.info.mapFunction->getCommand();
        // generate time pattern command
        helper.info.timePattern->update(command);
        command = helper.info.timePattern->getCommand();
        
        double normalizedCommand = command;
        double actuationIntensity = -1;
        if(normalizedCommand>0)
        {
            if(normalizedCommand>1.0) normalizedCommand = 1.0;

            actuationIntensity = normalizedCommand;
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
    bool checkGroupVelocity(const ActuatorGroupHelper& groupInfo)
    {
        for(const int &index : groupInfo.interfaceIndexes)
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
    double computeCommand(const ActuatorGroupHelper& groupInfo)
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

        contactsList[groupInfo.groupIndex] = norm>groupInfo.info.minThreshold ? 1 : 0;

        return computeActuationIntensity(groupInfo, norm);
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
        // iterate over enabled actuator groups
        auto listOfGroupsBottle = rf.find("list_of_groups").asList();
        for(int i=0; i<listOfGroupsBottle->size() ; i++)
        {
            std::string groupName = listOfGroupsBottle->get(i).asString();
            actuatorGroupNames.push_back(groupName);

            // parse the info object
            auto group = rf.findGroup(groupName);

            if(group.isNull())
            {
                yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Group"<< groupName << "not found!";
                return false;
            }

            if(!actuatorsGroupFactory.parseFromConfig(group))
            {
                yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << actuatorsGroupFactory.getParseError();
                return false;
            }
            WeightRetargeting::ActuatorsGroup& groupInfo = actuatorsGroupFactory.getGroup(groupName);

            // create the helper object
            actuatorGroupMap.insert({groupName, ActuatorGroupHelper(groupInfo)});
            ActuatorGroupHelper& groupHelper = actuatorGroupMap.at(groupName);
            
            groupHelper.groupIndex = i;
            groupHelper.offset = 0.0;

            // add joint names and indices to the list
            for(std::string& axisName : groupInfo.jointAxes)
            {
                auto it = std::find(jointNames.begin(), jointNames.end(), axisName);

                if(it==jointNames.end())
                {
                    jointNames.push_back(axisName);
                }

                //TODO no else, use groupHelper.jointIndexes
                if(retargetedValue==RetargetedValue::ForcePort)
                {
                    if(it==jointNames.end())
                    {
                        groupHelper.interfaceIndexes.push_back(jointNames.size()-1);
                    }
                    else
                    {
                        groupHelper.interfaceIndexes.push_back(it - jointNames.begin());
                    }
                }
            }

            // Set the indices of the buffer
            if(retargetedValue==RetargetedValue::ForcePort)
            {
                for(int j=0; j<3; j++)
                {
                    groupHelper.interfaceIndexes.push_back(i*3 + j);
                }
            }
        }

        if(retargetedValue==RetargetedValue::ForcePort)
        {
            interfaceValuesSize = listOfGroupsBottle->size() * 3;
        }
        else
        {
            interfaceValuesSize = jointNames.size();
        }

        return true;
    }

    /**
     * @brief Generates the actuation commands for all of the configured groups
     * 
     */
    void generateGroupsActuation()
    {
        for(auto & pair : actuatorGroupMap)
        {
            ActuatorGroupHelper& actuatorGroupHelper = pair.second;

            double actuationIntensity = computeCommand(actuatorGroupHelper);
            if(actuationIntensity<0) actuationIntensity = 0.0;
            if(actuationIntensity>minIntensity || (actuationIntensity<=0 && actuatorGroupHelper.lastCommand!=actuationIntensity))
            {
                //send the haptic command to all the related actuators
                for(const std::string& actuator : actuatorGroupHelper.info.actuators)
                { 
                    wearable::msg::WearableActuatorCommand& wearableActuatorCommand = actuatorCommandPort.prepare();

                    wearableActuatorCommand.value = actuationIntensity;
                    wearableActuatorCommand.info.name = IFEEL_SUIT_ACTUATOR_PREFIX+actuator;;
                    wearableActuatorCommand.info.type = wearable::msg::ActuatorType::HAPTIC;
                    wearableActuatorCommand.duration = 0;

                    // Send haptic actuator command
                    actuatorCommandPort.write(true);
                }
            }

            actuatorGroupHelper.lastCommand = actuationIntensity;
        }
    }

    bool acquireForcePortData(double *currs)
    {
        for(int i =0; i<actuatorGroupNames.size(); i++)
        {
            yarp::sig::Vector* input = forcePorts[i]->read(false);
        
            for(int j=0; j<3 ; j++)
            {
                currs[i*3+j] = input!=nullptr ? (*input)[j] : 0.0;
            }
        }

        return true;
    }

    bool updateModule() override
    {
        std::lock_guard<std::mutex> guard(mutex);
        auto currentTime = std::chrono::system_clock::now();
        
        // get the data 
        std::vector<double> buffer;
        bool acquisitionResult = false;
        buffer.resize(interfaceValuesSize);
        switch (retargetedValue)
        {
        case RetargetedValue::JointTorque : acquisitionResult = iTorqueControl->getTorques(buffer.data()); break;
        case RetargetedValue::MotorCurrent : acquisitionResult = iCurrentControl->getCurrents(buffer.data()); break;
        case RetargetedValue::ForcePort : acquisitionResult = acquireForcePortData(buffer.data()); break;
        default: acquisitionResult = false; break;
        } 

        // check if acquisition was successful
        if(acquisitionResult)
        {

            // apply the filter
            if(enableFilter)
            {
                interfaceValues = lowPassFilter.filt(buffer);
            } else
            {
                interfaceValues = buffer;
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

        if(useContacts)
        {
            yarp::sig::Vector& contactListMsg = contactListPort.prepare();
            contactListMsg = contactsList;
            contactListPort.write(false);
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
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter min_intensity, using default value" << minIntensity;
        } else 
        {
            minIntensity = rf.find("min_intensity").asFloat64();
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter min_intensity:" << minIntensity;
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
        
        // read cutoff frequency param
        if(!rf.check("cutoff_frequency"))
        {
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter cutoff_frequency, using default value" << cutoffFrequency;
        } else 
        {
            cutoffFrequency = rf.find("cutoff_frequency").asFloat64();
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter cutoff_frequency:" << cutoffFrequency;
        }

        // read enable_low_pass_filter
        if(!rf.check("enable_low_pass_filter"))
        {
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter enable_low_pass_filter, using default value" << enableFilter;
        } else
        {
            enableFilter = rf.find("enable_low_pass_filter").asBool();
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter enable_low_pass_filter:" << enableFilter;
        }

        // read publish_contacts
        if(!rf.check("publish_contacts"))
        {
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Missing parameter publish_contacts, using default value" << useContacts;
        } else
        {
            useContacts = rf.find("publish_contacts").asBool();
            yCIInfo(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Found parameter publish_contacts:" << useContacts;
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
        propRemapper.put("localPortPrefix", PORT_PREFIX+"/control_board_input");
        yarp::os::Property& remoteControlBoardsOpts = propRemapper.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
        remoteControlBoardsOpts.put("writeStrict", "off");

        // open the remapped control board
        //TODO find a better solution
        result = jointNames.size()==0 || remappedControlBoard.open(propRemapper);
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
        case RetargetedValue::ForcePort:
            result = true;
            for(int actuatorGroupIdx = 0; actuatorGroupIdx < actuatorGroupNames.size() ; actuatorGroupIdx++)
            {
                forcePorts.push_back(std::make_unique<yarp::os::BufferedPort<yarp::sig::Vector>>());
                std::string portName = PORT_PREFIX + "/force_ports/"+actuatorGroupNames[actuatorGroupIdx]+":i";
                result = forcePorts[actuatorGroupIdx]->open(portName);
            }
            break;
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

        interfaceValues.resize(interfaceValuesSize);
        velocities.resize(jointNames.size());

        std::fill(interfaceValues.begin(),interfaceValues.end(), 0.0);

        // Init filter
        lowPassFilter.init(cutoffFrequency, 1/period, interfaceValuesSize);

        std::string wearableActuatorCommandPortName = PORT_PREFIX+"/output:o";//TODO config

        // Initialize actuator command port and connect to command input port
        if(!actuatorCommandPort.open(wearableActuatorCommandPortName))
        {
            yCIError(WEIGHT_RETARGETING_LOG_COMPONENT, LOG_PREFIX) << "Failed to open" << actuatorCommandPort.getName();
            return false;
        }

        if(useContacts)
        {
            contactListPort.open(PORT_PREFIX+"/contacts:o");
            contactsList.resize(actuatorGroupNames.size());
            contactsList.zero(); 
        }

        // Initialize RPC
        this->yarp().attachAsServer(rpcPort);
        std::string rpcPortName = PORT_PREFIX+"/rpc:i"; //TODO from config?
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

        for(auto & forcePort : forcePorts)
        {
            forcePort->close();
        }

        if(useContacts)
        {
            contactListPort.close();
        }

        return true;
    }

    bool setMaxThreshold(const std::string& actuatorGroup, const double value) override
    {
        std::lock_guard<std::mutex> guard(mutex);
        if(actuatorGroupMap.find(actuatorGroup)==actuatorGroupMap.end())
            return false;

        actuatorGroupMap.at(actuatorGroup).info.maxThreshold = value; //TODO use update method
        return true;
    }

    bool setMinThreshold(const std::string& actuatorGroup, const double value) override
    {
        std::lock_guard<std::mutex> guard(mutex);
        if(actuatorGroupMap.find(actuatorGroup)==actuatorGroupMap.end())
            return false;

        actuatorGroupMap.at(actuatorGroup).info.minThreshold = value;//TODO use update method
        return true;
    }

    bool setThresholds(const std::string& actuatorGroup, const double minThreshold, const double maxThreshold) override
    {
        std::lock_guard<std::mutex> guard(mutex);
        if(actuatorGroupMap.find(actuatorGroup)==actuatorGroupMap.end())
            return false;

        actuatorGroupMap.at(actuatorGroup).info.minThreshold = minThreshold;
        actuatorGroupMap.at(actuatorGroup).info.maxThreshold = maxThreshold;
        return true;
    }

    void removeSingleOffset(const std::string& actuatorGroup)
    {
        ActuatorGroupHelper& groupHelper = actuatorGroupMap.at(actuatorGroup);

        groupHelper.offset = groupHelper.info.minThreshold - getNorm(groupHelper);
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
