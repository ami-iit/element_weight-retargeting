# WeightRetargetingModule
## Configuration file

The module requires the following parameters, which can be passed via a `.ini` configuration file:

| Name                | Description                                                                                                                                                                                                    | Example                                     |
|---------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------|
| robot               | Prefix of the yarp ports published by the robot                                                                                                                                                                | "icub"                                     |
| remote_boards       | List of the remote control boards that publish the data                                                                                                                                                        | ("left_arm" "right_arm")                  |
| actuator_groups | List of parameters related to actuator groups.   Each element of the list is a sublist: (\<group-name> \<joint-axis-name>  \<min-torque-thresh> \<max-torque-thresh> \<list-of-retargeted-actuators>)  | (("left_biceps" "l_elbow"  0.5 5.5 ( "13@1"   "13@2" ))) |

An example of configuration file is [`WeightRetargetingElbows.ini`](conf/WeightRetargetingElbows.ini).

**NOTE** setting a minimum threshold greater than the max one, will have the effect of using the joint torques in the negative direction. This can be helpful when wanting to retarget negative torques (see [conf/WeightRetargetingElbows.ini](conf/WeightRetargetingElbows.ini)) for an example.

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
|                 | 1: actuatorGroup| The name of the group of actuators (e.g. "left_biceps")             |
|                 | 2: value        | The value of the maximum threshold                      |
|                 |          |             |
| setMinThreshold |                 | Sets the maximum joint torque threshold of a joint axis |
|                 | 1: actuatorGroup| The name of the group of actuators (e.g. "left_biceps")             |
|                 | 2: value        | The value of the minimum threshold                      |
|                 |          |             |
| setThresholds   |                 | Set the minimum and maximum joint torque thresholds     |
|                 | 1: actuatorGroup| The name of the group of actuators (e.g. "left_biceps")             |
|                 | 2: minThreshold | The value of the minimum threshold                      |
|                 | 3: maxThreshold | The value of the maximum threshold                      |

An example of how to use the RPC:
```bash
yarp rpc /WeightRetargeting/rpc:i
setThresholds left_biceps 1.0 1.2
```



The message `Response: [ok]` will be shown if the operation was successful.

An useful approach is to set the thresholds before connecting the ports mentioned below. If not set properly, the actuators may start vibrating when not wanted.  
The module logs information about the commands sent and the values of the joint torques used, which can be used for this purpose. An example:

```bash
[INFO] |WeightRetargetingModule| Sending 16 to group right_triceps with r_elbow torque 1.13051
[INFO] |WeightRetargetingModule| Not actuating the group left_triceps , l_elbow torque is 1.2205
[INFO] |WeightRetargetingModule| Sending 15 to group right_triceps with r_elbow torque 1.12559
[INFO] |WeightRetargetingModule| Not actuating the group left_triceps , l_elbow torque is 1.2205
```

# WeightDisplayModule

## Configuration file

| Name                | Description  | Example | Required |
|---------------------|-----------------------------------------------------------------|---------------------------------------------|---|
| period               | Working frequency of the module in seconds                                                                                                                                                                | 0.05                                     | :x: 
| port_prefix       | Prefix of the YARP ports opened by the module                                                                                                                                                        | /WeightDysplayModule                  | :x:|
| min_weight | Minimum weight to be displayed in kilograms | 0.1 |:x: |
| input_port_names| Names of the ports opened by the module to read the end-effector wrenches | (left_hand right_hand) | :heavy_check_mark:|
| rpc_port | Name of the rpc port to enable the text label related to the weight | /joypadDevice/Oculus/rpc | :x: | 
| label_id | Id of the weight text label | 0 | Required if rpc_port is specified |

## Running the module

The following steps assume that installation files are visible by YARP (see [Configure the enviroment](Installation.md#configure-the-environment)).

The module can be run with the following command:

```bash
WeightDisplayModule --from WeightDisplay.ini
```

Alternatively, the module can be run via [`yarpmanager`](https://www.yarp.it/latest//yarpmanager.html) through the application [`iFeelSuitWeightRetargeting`](apps/iFeelSuitWeightRetargeting.xml).

Once the module has started, it starts publishing the weight of objects being held by the robot via the output port `<port_prefix>/out:o` as a text, assuming that the input ports have been connected to the ones where the corresponding wrenches are published.

In order to let the weight be shown via the OpenXR module, connect the input port of the text label related to the weight to the output port of the WeightDisplayModule. 
This can be done either via `yarpmanager` or via command-line:
```bash
yarp connect /WeightDisplay/out:o joypadDevice/Oculus/label_<label_ID>
```