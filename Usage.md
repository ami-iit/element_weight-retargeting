# WeightRetargetingModule

## How it works

The module holds a data structure linking group of actuators to a list of associated joints. It periodically retrieves some values for such joints (e.g motor current) and compute their square norm. This norm is compared against a minimum and a maximum threshold in order to compute a linear mapping towards the value of the actuation command.
The module can also use joint velocity information to disable the generation of the actuation command, so that if the velocity of the joints related to an actuator group is above some threshold, it won't be considered.

## Configuration file

The module requires the following parameters, which can be passed via a `.ini` configuration file:

| Name                | Description                                                                                                                                                                                                    | Example                                     |
|---------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------|
| robot               | Prefix of the yarp ports published by the robot                                                                                                                                                                | "icub"                                     |
| retargeted_value | Value of the joints to be used for the retargeting. Eligible values are "motor_currents" and "joint_torques". | motor_current |
| remote_boards       | List of the remote control boards that publish the data                                                                                                                                                        | ("left_arm" "right_arm")                  |
| actuator_groups | List of parameters related to actuator groups.   Each element of the list is a sublist: (\<group-name> \<list-of-joint-axis-names>  \<min-value-thresh> \<max-value-thresh> \<list-of-retargeted-actuators>)  | (("left_arm" ("l_wrist_pitch" "l_wrist_yaw") 0.45 1.5 ("13@1" "13@2" "13@4"))) |
| min_intensity | Minimum actuation intensity that is sent by the module | 20.0 |
| use_velocity | Flag for checking the joints velocities to allow the retargeting | true |
| max_velocity | Max velocity for a group's joint to allow the haptic retargeting in rad/s| 0.15 |

An example of configuration file is [`WeightRetargeting_iCub3.ini`](conf/WeightRetargeting_iCub3.ini).

## Running the module

The following steps assume that installation files are visible by YARP (see [Configure the environment](Installation.md#configure-the-environment)).

The module can be run with the following command:

```bash
WeightRetargetingModule --from WeightRetargeting_iCub3.ini
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

The module provides with an RPC service accessible via the port `/WeightRetargeting/rpc:i` that allows to change the thresholds in real-time. Below, the specification of the implemented methods

| Method            | Parameters      | Description                                             |
|-----------------|-----------------|---------------------------------------------------------|
| setMaxThreshold |                 | Sets the maximum group threshold of a joint axis |
|                 | 1: actuatorGroup| The name of the group of actuators (e.g. "left_biceps")             |
|                 | 2: value        | The value of the maximum threshold                      |
|                 |          |             |
| setMinThreshold |                 | Sets the maximum group threshold of a joint axis |
|                 | 1: actuatorGroup| The name of the group of actuators (e.g. "left_biceps")             |
|                 | 2: value        | The value of the minimum threshold                      |
|                 |          |             |
| setThresholds   |                 | Set the minimum and maximum group thresholds     |
|                 | 1: actuatorGroup| The name of the group of actuators (e.g. "left_biceps")             |
|                 | 2: minThreshold | The value of the minimum threshold                      |
|                 | 3: maxThreshold | The value of the maximum threshold                      |
| | |
| removeOffset | | Remove the offset from the current value to the minimum threshold of the group |
| |1: actuatorGroup | The name of the interested group (e.g. "left_arm"). Name `all` can be used for removing the offset of all of the configured groups.|

An example of how to use the RPC:
```bash
yarp rpc /WeightRetargeting/rpc:i
setThresholds left_arm 1.0 1.2
```

The message `Response: [ok]` will be shown if the operation was successful.

# WeightDisplayModule

## How it works

The module reads data from a specified list of YARP ports publishing the wrenches externally exerted on the robot's end effectors (i.e. the hands).
This wrenches are used to compute the weight of the object the robot is holding, which is published as text via a YARP port.

The module can also use joint velocity information to exclude the use of some wrenches. If the option is enabled, wrenches associated to a joint with a velocity above threshold won't be considered for the computation of the weight. 


## Configuration file

| Name                | Description  | Example | Required |
|---------------------|-----------------------------------------------------------------|---------------------------------------------|---|
| period               | Working frequency of the module in seconds                                                                                                                                                                | 0.05                                     | :x: 
| port_prefix       | Prefix of the YARP ports opened by the module                                                                                                                                                        | /WeightDisplayModule                  | :x: |
| min_weight | Minimum weight to be displayed in kilograms | 0.1 | :x: |
| input_port_names| Names of the ports opened by the module to read the end-effector wrenches | (left_hand right_hand) | :heavy_check_mark: |
| | | |
|VELOCITY_UTILS| A parameter group with info for checking the joints velocities | | :x: |
| use_velocity | Flag for enabling the joint velocity check (default `false`) | true | :x: |
| max_velocity | Max joint velocity value. If one of the joints velocities is above this threshold, the related wrench is not accounted for the weight computation | 0.15 | If `use_velocity` is true |
| robot | Prefix of the yarp ports published by the robot | "icub" | If `use_velocity` is true |
| remote_boards | List of the remote control boards that publish the joint velocity data | ("left_arm" "right_arm") | If `use_velocity` is true |
| joints_info | List of wrench port to joint associations. The association are lists in the form ( <port_name> <joint_axis_1> .. <joint_axis_n> ) | (("left_hand" "l_wrist_pitch" "l_wrist_yaw")) | If `use_velocity` is true |



## Running the module

The following steps assume that installation files are visible by YARP (see [Configure the environment](Installation.md#configure-the-environment)).

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

## :warning: Usage notes 

The module uses the wrenches read from the input ports to compute the weight. 
In order to do that it assumes that the reference frames of the wrenches have the z-axis orthogonal w.r.t. the ground and the direction opposite to it.
