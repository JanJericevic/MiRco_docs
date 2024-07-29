# MiRco ROS interface control
Before controlling `MiRco` it is recommended to read the [MiRco interface](./mirco_interface.md) chapter of the documentation.

Make sure to follow the **ROS interface** parts of the [Starting `MiRco`](./mirco_start.md) instructions.

## Packages overview
- **`mirco_robot:`** main control package
- **`mirco_description:`** URDF description and launch files
- **`mirco_moveit_config:`** MoveIt! configuration and launch files

For an overview of the individual ROS packages for `MiR100` and `UR5e` see the [`MiR100`](../MiR100/mir_ros_control.md) and [`UR5e`](../UR5e_2F85/ur_2f85_ros_control.md) ROS control documentation.

## Usage
### Start MiRco control
- Prepare the robot arm. On the teach pendant load the [external control UR program](./mirco_start.md/#external-control-ur-program) program. Power on and start the robot manipulator.

- Run `mirco_bringup.launch`. Wait until the "subscribing to" & "publishing to" messages stop appearing. This means the connection is established
```bash
# MiR100 connection over hotspot
roslaunch mirco_robot mirco_bringup.launch

# MiR100 connection over outside network
roslaunch mirco_robot mirco_bringup.launch mir_ip:=<MiR100_IP>

```
If you know `MiR100_IP` will not be changing (if you set up static IP) you can edit `mirco_bringup.launch` as:
```bash
# line 7
<arg name="mir_ip" default="<MiR100_IP>" doc="MiR IP address"/>

# then, while connected to an outside network, you can run 
roslaunch mirco_robot mirco_bringup.launch
```

- Start the external control UR program on the teach pendant. In the terminal where you launched `mirco_bringup` you should see:
```
Robot connected to reverse interface. Ready to receive control commands
```

- In another terminal run `mirco_control.launch`. 
```bash
roslaunch mirco_robot mirco_control.launch
```

### RViz
After you started MiRco control, you can send the robot to a goal using the `2D Nav Goal` tool in RViz.  

Send the robot to a goal:

- open rviz
```
roslaunch mirco_robot view_mirco.launch
```
- Make sure these RViz settings are set:
```
Panels -> Tool Properties -> 2D Nav Goal -> Topic: /mirco/mir100/move_base_simple/goal
Panels -> Tool Properties -> 2D Pose Estimate -> Topic: /mirco/mir100/initialpose
Global Options -> Fixed Frame: map
```
- use `2D Pose Estimate` to mark the approximate MiR100 start location
- use `2D Nav Goal` to send the robot to a goal

TODO: check rviz config

## mirco_robot package
Overview of the main control package.

### Launch files
**mirco_bringup.launch**

- sets `MiRco`, `MiR100`, `UR5e` and `2F-85` namespaces and TF prefixes
- sets robot IPs for the robot drivers
- loads ROS parameters and `MiRco` URDF model
- establishes connection with `MiR100`, `UR5e` and `2F-85`

**mirco_control.launch**

- launches `mir_control_node`
- launches !MoveIt for the manipulator

**ur_control.launch:** Part of the `UR5e` ROS driver

**view_mirco.launch:** Launches RViz with saved config

**delivery.launch**

Demo delivery application inside Robolab. Needs (TODO:map name) to be the active `MiR100` map. Expects the use of the VL docking marker. 

- delivers a gear to a station
- places the gear on the station
- goes away
- returns to the station and picks up the gear
- goes to an end position 

**conveyor.launch**: Demo pick and place application for the manipulator. Same as `delivery.launch` without repositioning of the mobile base.

### Config files
#### arm_gripper_joint_sources.yaml
`MiRco` uses 2 `joint_state_publisher` nodes. 

One collects the sepparate joint states information of `UR5e` and `2F-85`. It then combines them and publishes them to the `/mirco_namespace/ur5e/joint_states` topic.

The other collects the combined joint states information of `UR5e` and `2F-85` (from the first publisher) and the joint states of `MiR100`. It then publishes them to the `/mirco_namespace/joint_states` topic.

This confg file contains sources for where the first publisher looks for this joint information.

#### robot_joint_sources.yaml
`MiRco` uses 2 `joint_state_publisher` nodes. 

One collects the sepparate joint states information of `UR5e` and `2F-85`. It then combines them and publishes them to the `/mirco_namespace/ur5e/joint_states` topic.

The other collects the combined joint states information of `UR5e` and `2F-85` (from the first publisher) and the joint states of `MiR100`. It then publishes them to the `/mirco_namespace/joint_states` topic.

This confg file contains sources for where the second publisher looks for this joint information.

#### ur5e_saved_poses.yaml
File used to save manipulator poses as joint states. Used by `save_arm_pose`, `set_arm_pose`, `get_arm_pose` and `get_arm_pose_names` services to save or retrieve manipulator joint states.

```yaml
# saved pose example
entry1:
  ur5e_elbow_joint: -1.9254509210586548
  ur5e_shoulder_lift_joint: -1.3431395900300522
  ur5e_shoulder_pan_joint: -0.07610828081239873
  ur5e_wrist_1_joint: -1.4239323896220704
  ur5e_wrist_2_joint: 1.5782090425491333
  ur5e_wrist_3_joint: -0.07931977907289678
```

### Services

- **[`MiR100` services](../MiR100/mir_ros_control.md/#services)**
- **[`UR5e` services](../UR5e_2F85/ur_2f85_ros_control.md/#services)**