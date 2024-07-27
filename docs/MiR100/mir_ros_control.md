# ROS control
This page describes the ROS control for the **MiR100 robot only** . To control the mobile base as part of the MiRco mobile manipulator see [MiRco ROS control](../MiRco/mirco_ros_control.md).  

To establish a ROS connection with MiR100 see [Connect to MiR100](./mir_connection.md).

!!! warning
    MiR100 internal clock is prone to desynchronization which can mess with the robot control.  
    Before controlling the robot, you have to connect to the [MiR web interface](./mir_connection.md/#establish-mir-web-interface-connection) and go to `System -> Settings -> Date & Time` and synchronize the internal clock. Click `Load from device` and `Save changes`.

!!! note
    For an advanced setup for clock synchronization see this [solution](https://github.com/DFKI-NI/mir_robot?tab=readme-ov-file#advanced).

## Packages overview
- **`mir_control:`** main control package
- **`mir_rest_api:`** [MiR REST API](./mir_rest_api.md)
- **`mir_robot:`** MiR ROS driver developed by the [DFKI team](https://github.com/DFKI-NI/mir_robot)
- **`mir_joy_teleop:`** joystick control

## Usage
### Start mir_control
```bash
# hotspot
roslaunch mir_control mir_control.launch

# outside network
roslaunch mir_control mir_control.launch mir_ip:=<MiR100_IP>
```  

### RViz
TODO: check, make rviz config

After you started `mir_control`, you can send the robot to a goal using the `2D Nav Goal` tool in RViz.  

Make sure to set these RViz settings:

```
Panels -> Tool Properties -> 2D Nav Goal -> Topic: /mir_namespace/move_base_simple/goal
Panels -> Tool Properties -> 2D Pose Estimate -> Topic: /mir_namespace/initialpose
Global Options -> Fixed Frame: map
```

Send the robot to a goal:

- use `2D Pose Estimate` to mark the approximate MiR100 start location
- use `2D Nav Goal` to send the robot to a goal

## mir_control package
The main control package. This is a wrapper around the `mir_driver` package. For details about `mir_driver` see the [GitHub page](https://github.com/DFKI-NI/mir_robot).

`mir_control` starts another `roscore` on the host computer, connecting to the MiR100 internal `roscore` over `ROS Bridge`. See the [MiRco ROS schematic](../MiRco/mirco_interface.md/#mirco-ros-interface) for a visual aid of the connection.  

`mir_control` adds a control node, a python robot class, a goal teacher, ROS services and [REST API integration](./mir_rest_api.md). This effectively enables the use of all web interface functions.

### Launch files 
#### `mir_control.launch`
- launches `mir_driver` 
- launches `mir_control_node`


### Nodes 
#### `mir_control_node.py` 
The main control node for MiR100. Initializes a `MiR100` python object. See `mir100_class.py` script for details on the object.

### Config files
#### target_goals.yaml
When you run the control node, the positions in the current MiR map are saved in this file.  
If `taget_goals.yaml` already contains a position with the same name (this file is not reset on start), the old position is overwritten.

```yaml
# saved positions structure
position_name:
  orientation:
    w: 0.0112920402716304
    x: 0.0
    y: 0.0
    z: -0.9999362428807668
  position:
    x: 21.218
    y: 13.802
    z: 0      # z coordinate is always saved as 0
```


#### markers.yaml
When you run the control node, the markers in the current MiR map are saved in this file.  
This doesn't include marker entry positions. Currently only VL type markers are supported.  
If `markers.yaml` already contains a marker with the same name (this file is not reset on start), the old marker is overwritten.

```yaml
# saved markers structure
marker_name:
  guid: 017f3480-fb3d-11ee-947c-94c691a73828
  offsets_guid: 018037c1-fb3d-11ee-947c-94c691a73828
  orientation_offset: 2.0
  type_id: 11
  x_offset: -0.6
  y_offset: 0.0
```

### Services
Services are established with the `mir_control_node` or whenever you create a `MiR100` object defined in `mir100_class.py`.

#### save_mobile_goal (TODO: check if works correctly)
Saves the current MiR100 pose under selected name. The pose is saved in `/config/target_goals.yaml`.  

If the current MiR100 map does not include a position or marker with the same name, a position with the selected name is created and saved to the current MiR map. If a position or marker with the same name already exists, they are overwritten with the current MiR100 pose.

#### send_to_goal
Send robot to a goal saved in `/config/target_goals.yaml`.  

#### get_markers
Get list of markers saved in `/config/markers.yaml`.

#### dock_to_vl_marker
Start docking to the selected marker saved in `config/markers.yaml`.

#### change_marker_offsets
Change the offsets of a marker saved in `/config/markers.yaml`.  
Currently only VL type markers are supported.  

## mir_joy_teleop package
A package for joystick control.

As with [connecting to the robot](./mir_connection.md/#establish-ros-connection), you can send the joystick commands directly to the MiR100 internal `roscore` or use it with the `mir_control` package.

!!! note
    
    When using `mir_control` we send `geometry_msgs/Twist` type messages instead of `geometry_msgs/TwistStamped` to the `/cmd_vel` topic. That is because the `mir_driver` package expects messages of type `geometry_msgs/Twist` on the `/cmd_vel` topic and converts them to `geometry_msgs/TwistStamped` messages before sending the commands to the robot.

### Internal 
Follow the [Establish ROS connection - internal roscore](./mir_connection.md/#internal-roscore) instructions.  
Then run:

```bash
# launch a joy_node and a teleop node
roslaunch mir_joy_teleop joy_teleop.launch

# default input device is js1
# you can specify input device - e.g. js2
$ roslaunch mir_joy_teleop joy_teleop.launch device:=js2
```

### mir_control package
Follow the [Establish ROS connection - mir_control package](./mir_connection.md/#mir_control-package) instructions.  
Then run:
```bash
# launch a joy_node and a teleop node
roslaunch mir_joy_teleop joy_teleop.launch roscore:=external

# default input device is js1
# specify input device - e.g. js2
$ roslaunch mir_joy_teleop joy_teleop.launch device:=js2 roscore:=external
```