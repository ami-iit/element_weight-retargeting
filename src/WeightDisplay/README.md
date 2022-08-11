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
