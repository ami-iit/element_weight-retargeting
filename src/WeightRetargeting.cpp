#include <iostream>

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ITorqueControl.h>

#include <thrift/WearableActuatorCommand.h>

#define WEIGHT_RETARGETING_MAX_INTENSITY 123

class WeightRetargetingModule : public yarp::os::RFModule
{
public:

    yarp::dev::PolyDriver remappedControlBoard;
    yarp::dev::ITorqueControl* iTorqueControl{ nullptr };

    std::vector<std::string> remoteControlBoards;
    std::vector<std::string> jointNames;
    std::vector<std::vector<std::string>> jointToActuators;
    std::vector<double> jointTorquesMinThresholds;
    std::vector<double> jointTorquesMaxThresholds;

    // Haptic command
    yarp::os::BufferedPort<wearable::msg::WearableActuatorCommand> actuatorCommandPort;

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

                //send the haptic command
                wearable::msg::WearableActuatorCommand& wearableActuatorCommand = actuatorCommandPort.prepare();

                wearableActuatorCommand.value = actuationIntensity;
                wearableActuatorCommand.info.type = wearable::msg::ActuatorType::HAPTIC;
                wearableActuatorCommand.info.status = wearable::msg::ActuatorStatus::OK;
                wearableActuatorCommand.duration = 0;
                
                for(std::string& actuator : jointToActuators[i])
                { 
                    wearableActuatorCommand.info.name = "iFeelSuit::haptic::"+actuator;

                    yInfo() << "Sending "<< actuationIntensity << " to " << actuator << " with joint torque " << jointTorques[i];

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
        if(!robotName.empty())
            yDebug()<<"robot OK";
        else
            yDebug() <<"robot NOT OK";

        if(rf.check("remote_boards"))
            yDebug()<<"remote_boards OK";
        else
            yDebug() <<"remote_boards NOT OK";

        if(rf.check("joints_to_actuators"))
            yDebug()<<"joints_to_actuators OK";
        else
            yDebug()<<"joints_to_actuators NOT OK";

        //TODO set from config
        remoteControlBoards = {"/right_arm"};
        jointNames = {"r_elbow"};
        jointToActuators = {{"Node#14@2", "Node#14@3"}};
        jointTorquesMinThresholds = {0.5}; //TODO use iTorqueControl->getTorqueRanges???
        jointTorquesMaxThresholds = {5.5}; //TODO use iTorqueControl->getTorqueRanges???

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
