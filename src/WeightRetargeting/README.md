# WeightRetargetingModule

## How it works

The module holds a data structure linking groups of actuators to a list of associated joints. It periodically retrieves some values for such joints (e.g motor current) and computes their square norm. This norm is compared against a minimum and a maximum threshold in order to compute a linear mapping towards the value of the actuation command.
The module can also use joint velocity information to disable the generation of the actuation command, so that if the velocity of the joints related to an actuator group is above some threshold, it won't be considered.

## Configuration file

The module requires the following parameters, which can be passed via a `.ini` configuration file:

| Name                | Description                                                                                                                                                                                                    | Example                                     |
|---------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------|
| robot               | Prefix of the yarp ports published by the robot                                                                                                                                                                | "icub"                                     |
| retargeted_value | Value of the joints to be used for the retargeting. Eligible values are `motor_current` and `joint_torque`. | "motor_current" |
| remote_boards       | List of the remote control boards that publish the data                                                                                                                                                        | ("left_arm" "right_arm")                  |
| min_intensity | Minimum actuation intensity that is sent by the module | 20.0 |
| use_velocity | Flag for checking the joints velocities to allow the retargeting | true |
| max_velocity | Max velocity for a group's joint to allow the haptic retargeting in rad/s| 0.15 |
| list_of_groups | List of actuators group names (see the [related section](#actuators-group-definition)) |  |


:warning: The value `all` cannot be used for an actuators group name.

An example of configuration file is [`WeightRetargeting_iCub3.ini`](conf/WeightRetargeting_iCub3.ini).

### Actuators group definition

For each name in the parameter `list_of_groups`, a group with the same name must be specified.
The group must have the following parameters:

| Name                | Mandatory | Description                                                                                                                                                                                                    | Example                                     |
|---------------------|--|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------|
| joint_axes | :heavy_check_mark: | The list of joint axes associated with this group of actuators | ("l_wrist_pitch" "l_wrist_yaw") |
| actuators | :heavy_check_mark: | The list of the actuators associated with this group | ("13@1" "13@2" "13@4")|
| | | |
| map_function | :heavy_check_mark: | Function that maps the retargeted value into a normalized actuation value. Eligible values are `linear` and `steps`. | "linear"|
|min_threshold| :heavy_check_mark: | Minimum input value accepted by the function| 0.45 | 
|max_threshold| :heavy_check_mark: | Maximum input value accepted by the function| 1.5 |
| steps_number | if `map_function`=`steps` and `steps_thresholds`,`steps_commands` are not defined| Number of steps of the map function | 5 | 
| steps_thresholds | if `map_function`=`steps` and `steps_number` is not defined| List of input thresholds that define the steps |(0.2 0.5 0.8)|
|steps_commands | if `map_function`=`steps` and `steps_number` is not defined | Normalized commands associated with each step | (0.4 0.5 0.8 1.0) |
| | | |
|time_pattern | :heavy_check_mark: | Function that handles the actuation in time depending on the value returned by the map function. Eligible values are `continuous` and `pulse`| "continuous" |
|pulse_pattern_custom_actuation | :x: | The actuation value of the pulse. If negative (default) use the value given as input instead.| 0.5 |
|pulse_pattern_levels | if `time_pattern`=`pulse`| The number of levels that determine the number of pulses per minute.| 3|
|pulse_pattern_max_frequency | if `time_pattern`=`pulse` | The frequency of the pulses expressed in Hz | 0.5 |
|pulse_pattern_thresholds | if `time_pattern`=`pulse` and `pulse_pattern_levels` is not defined| The thresholds used to define the start of the levels| (0.2 0.4 0.9)|
|pulse_pattern_frequencies | `time_pattern`=`pulse` and `pulse_pattern_levels` is not defined | The frequencies associated to the different levels, expressed in Hz| (0.5 1 2) |


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

**NOTE**: `WeightRetargeting_iCub3.ini` is an example of configuration file.

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
