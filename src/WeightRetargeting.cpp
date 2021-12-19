#include <algorithm>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ITorqueControl.h>

#include <thrift/WeightRetargetingService.h>

#include <thrift/WearableActuatorCommand.h>

#define WEIGHT_RETARGETING_MAX_INTENSITY 127

class WeightRetargetingModule : public yarp::os::RFModule, WeightRetargetingService
{
public:

    const std::string IFEEL_SUIT_ACTUATOR_PREFIX = "iFeelSuit::haptic::Node";

    yarp::dev::PolyDriver remappedControlBoard;
    yarp::dev::ITorqueControl* iTorqueControl{ nullptr };

    std::vector<std::string> remoteControlBoards;
    std::vector<std::string> jointNames;
    std::vector<std::vector<std::string>> jointAxesToActuators;
    std::vector<double> jointTorquesMinThresholds;
    std::vector<double> jointTorquesMaxThresholds;

    // Haptic command
    yarp::os::BufferedPort<wearable::msg::WearableActuatorCommand> actuatorCommandPort;

    // RPC
    yarp::os::Port rpcPort;

    double * jointTorques = nullptr;

    double getPeriod() override
    {
        return 0.02; //50Hz
    }

    bool updateModule() override
    {
        iTorqueControl->getTorques(jointTorques);

        for(int i = 0; i<jointNames.size(); i++)
        {
            // compute values
            double normalizedIntensity = (jointTorques[i] - jointTorquesMinThresholds[i]) / (jointTorquesMaxThresholds[i] - jointTorquesMinThresholds[i]);

            if(normalizedIntensity>0)
            {
                if(normalizedIntensity>1.0) normalizedIntensity = 1.0;

                //TODO check if it's better to use steps
                double actuationIntensity = (int)(normalizedIntensity*WEIGHT_RETARGETING_MAX_INTENSITY);

                //send the haptic command to all the related actuators
                for(std::string& actuator : jointAxesToActuators[i])
                { 
                    wearable::msg::WearableActuatorCommand& wearableActuatorCommand = actuatorCommandPort.prepare();

                    wearableActuatorCommand.value = actuationIntensity;
                    wearableActuatorCommand.info.type = wearable::msg::ActuatorType::HAPTIC;
                    wearableActuatorCommand.info.status = wearable::msg::ActuatorStatus::OK;
                    wearableActuatorCommand.duration = 0;

                    wearableActuatorCommand.info.name = IFEEL_SUIT_ACTUATOR_PREFIX+actuator;

                    yInfo() << "Sending "<< wearableActuatorCommand.value << " to " << wearableActuatorCommand.info.name << " with joint torque " << jointTorques[i];

                    // Send haptic actuator command
                    // NOTE: Use strict flag true for writing all the commands without dropping any old commands
                    actuatorCommandPort.write(true);
                }
            }
            else
            {
                yInfo() << "Not sending command for joint " << jointNames[i] << ", the value is "<< jointTorques[i];
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
            yError() << "Missing parameter: robot";
            return false;
        }

        yarp::os::Bottle* remoteBoardsBottle = rf.find("remote_boards").asList();
        
        // remote control boards 
        if(remoteBoardsBottle==nullptr)
        {
            yError()<<"Missing parameter: remote_boards";
            return false;
        }

        yDebug()<<"remote_boards OK";
        for(int i=0;i<remoteBoardsBottle->size();i++)
        {
            std::string remoteBoard = remoteBoardsBottle->get(i).asString();
            
            remoteControlBoards.push_back(remoteBoard);
            yDebug()<< "Added remote control board:" << remoteBoard;
        } 
        
        // information on joint axes and actuators
        yarp::os::Bottle* jointToActuatorsBottle = rf.find("joints_to_actuators").asList();
        if(jointToActuatorsBottle==nullptr)
        {
            yError()<<"Missing parameter: joints_to_actuators";
            return false;
        }

        yDebug()<<"joints_to_actuators OK";
        for(int i=0; i<jointToActuatorsBottle->size(); i++)
        {
            yarp::os::Bottle* jointAxisInfo = jointToActuatorsBottle->get(i).asList(); 
            
            //get axis name
            std::string axisName = jointAxisInfo->get(0).asString();
            //get min threshold
            double minThresh = jointAxisInfo->get(1).asDouble();
            //get max threshold
            double maxThresh = jointAxisInfo->get(2).asDouble();
            //get list of actuators
            yDebug()<< (axisName.empty() ? "Axis name error!" : "Added joint axis "+axisName)
                    <<"| Min threshold"<< minThresh << "| Max threshold"<< maxThresh;
            yarp::os::Bottle* actuatorListBottle = jointAxisInfo->get(3).asList();
            std::vector<std::string> actuatorList = {};
            for(int j = 0; j<actuatorListBottle->size(); j++) actuatorList.push_back(actuatorListBottle->get(j).asString());

            jointNames.push_back(axisName);
            jointAxesToActuators.push_back(actuatorList);
            jointTorquesMinThresholds.push_back(minThresh);
            jointTorquesMaxThresholds.push_back(maxThresh);
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
            yError() << "Unable to open the ControlBoardRemapper";
            return result;
        }

        result = remappedControlBoard.view(iTorqueControl);

        if(!result)
        {
            yError() << "Unable to get torque control interface";
            return result;
        }

        int axes = 0;
        iTorqueControl->getAxes(&axes);
        if(axes!=jointNames.size())
        {
            //TODO
            yError() << "Number of iTorqueControl axes is different than the configured ones";
            result = false;
        }
        else
        {
            jointTorques = new double[axes];
        }

        std::string wearableActuatorCommandPortName = "/WeightRetargeting/output:o";//TODO config

        // Initialize actuator command port and connect to command input port
        if(!actuatorCommandPort.open(wearableActuatorCommandPortName))
        {
            yError() << "Failed to open " << actuatorCommandPort.getName();
            return false;
        }

        // Initialize RPC

        this->yarp().attachAsServer(rpcPort);
        std::string rpcPortName = "/WeightRetargeting/rpc:i"; //TODO from config?
        // open the RPC port
        if(!rpcPort.open(rpcPortName))
        {
            yError() << "Failed to open " << actuatorCommandPort.getName();
            return false;
        }
        // attach the port
        if (!this->yarp().attachAsServer(rpcPort)) {
            yError() << "Failed to attach " << rpcPortName << " to the RPC service";
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
        delete jointTorques;
        return true;
    }

    int findIndexOfAxis(const std::string& axis)
    {
        for(int i=0;i<jointNames.size(); i++)
            if(jointNames[i]==axis)
                return i;

        return -1;
    }

    bool setMaxThreshold(const std::string& axis, const double value) override
    {
        int index = findIndexOfAxis(axis);

        if(index==-1 || jointTorquesMinThresholds[index]>value)
            return false;

        jointTorquesMaxThresholds[index] = value;
        return true;
    }

    bool setMinThreshold(const std::string& axis, const double value) override
    {
        int index = findIndexOfAxis(axis);

        if(index==-1 || jointTorquesMaxThresholds[index]<value)
            return false;

        jointTorquesMinThresholds[index] = value;
        return true;
    }

    bool setThresholds(const std::string& axis, const double minThreshold, const double maxThreshold) override
    {
        if(minThreshold>=maxThreshold)
            return false;
        
        int index = findIndexOfAxis(axis);

        if(index==-1)
            return false;

        jointTorquesMinThresholds[index] = minThreshold;
        jointTorquesMaxThresholds[index] = maxThreshold;
        
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
