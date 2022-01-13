## Configuration file

The module requires the following parameters, which can be passed via a `.ini` configuration file:

| Name                | Description                                                                                                                                                                                                    | Example                                     |
|---------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------|
| robot               | Prefix of the yarp ports published by the robot                                                                                                                                                                | "/icub"                                     |
| remote_boards       | List of the remote control boards that publish the data                                                                                                                                                        | ("/left_arm" "/right_arm")                  |
| joints_to_actuators | List of joint-related parameters.   Each element of the list is a sublist: (\<joint-axis-name>  \<min-torque-thresh> \<max-torque-thresh> \<list-of-retargeted-actuators>)  | (( "l_elbow"  0.5 5.5 ( "13@1"   "13@2" ))) |

An example of configuration file is [`WeightRetargetingElbows.ini`](conf/WeightRetargetingElbows.ini).

## Running the module

The following steps assume that installation files are visible by YARP (see [Configure the enviroment](Installation.md#configure-the-environment)).

The module can be run with the following command:

```bash
WeightRetargetingModule --from WeightRetargetingElbows.ini
```

Alternatively, the module can be run via [`yarpmanager`](https://www.yarp.it/latest//yarpmanager.html) through the application [`iFeelSuitWeightRetargeting`](apps/iFeelSuitWeightRetargeting.xml).

Once the module has started, it will start publishing commands for the configured actuators.
In order to let the iFeelSuit receive the commands, connect the its input port to the output port of the WeightRetargetingModule. 
This can be done either via `yarpmanager` or via command-line:
```bash
yarp connect /WeightRetargeting/output:o /iFeelSuit/WearableActuatorsCommand/input:i
```

**NOTE**: `WeightRetargetingElbows.ini` is an example of configuration file which takes into account only the elbow joints.

## RPC 

The module provides with an RPC service accessible via the port `/WeightRetargeting/rpc:i` that allows to change the joint axes torques thresholds in real-time. Below, the specification of the implemented methods

| Method            | Parameters      | Description                                             |
|-----------------|-----------------|---------------------------------------------------------|
| setMaxThreshold |                 | Sets the maximum joint torque threshold of a joint axis |
|                 | 1: axis         | The name of the joint axis (e.g. "l_elbow")             |
|                 | 2: value        | The value of the maximum threshold                      |
|                 |          |             |
| setMinThreshold |                 | Sets the maximum joint torque threshold of a joint axis |
|                 | 1: axis         | The name of the joint axis (e.g. "l_elbow")             |
|                 | 2: value        | The value of the minimum threshold                      |
|                 |          |             |
| setThresholds   |                 | Set the minimum and maximum joint torque thresholds     |
|                 | 1: axis         | The name of the joint axis (e.g. "l_elbow")             |
|                 | 2: minThreshold | The value of the minimum threshold                      |
|                 | 3: maxThreshold | The value of the maximum threshold                      |

An example of how to use the RPC:
```bash
yarp rpc /WeightRetargeting/rpc:i
setThresholds r_elbow 1.0 1.2
```

The message `Response: [ok]` will be shown if the operation was successful.

An useful approach is to set the thresholds before connecting the ports mentioned below. If not set properly, the actuators may start vibrating when not wanted.  
The module logs information about the commands sent and the values of the joint torques used, which can be used for this purpose. An example:

```bash
[INFO] Sending  15  to  iFeelSuit::haptic::Node13@1  with joint torque  1.11062
[INFO] Sending  15  to  iFeelSuit::haptic::Node13@2  with joint torque  1.11062
[INFO] Sending  16  to  iFeelSuit::haptic::Node14@1  with joint torque  1.16912
[INFO] Sending  16  to  iFeelSuit::haptic::Node14@2  with joint torque  1.16912
```
