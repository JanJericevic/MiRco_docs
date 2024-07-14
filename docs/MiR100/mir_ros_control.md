# ROS control
This page describes the ROS control of **only** the MiR100 robot. To control the mobile base as part of the MiRco mobile manipulator see [MiRco control](../MiRco/mirco_control.md).  

To establish a ROS connection see [Establish ROS connection](./mir_connection.md/#establish-ros-connection)

!!! warning
    MiR100 internal clock is prone to desynchronization which can mess with the robot control.  
    Before controlling the robot, you have to connect to the [MiR web interface](./mir_connection.md/#establish-mir-web-interface-connection) and go to `System -> Settings -> Date & Time` and synchronize the internal clock. Click "load from device" and "Save changes".

!!! note
    For an advanced setup for clock synchronization see this [solution](https://github.com/DFKI-NI/mir_robot?tab=readme-ov-file#advanced).


## Package overview
MiR100 ROS control is done using the `mir_control` package. This is a wrapper around the `mir_driver` package. `mir_control` starts another `roscore` on the host computer, connecting to the MiR100 internal `roscore` over `ROS Bridge`. See the [MiRco ROS schematic](../MiRco/mirco_interface.md/#mirco-ros-interface) for a visual aid. It adds a control node, a python robot class, a goal teacher, ROS services and REST API integration. 

### Usage
```bash
# hotspot
roslaunch mir_control mir_control.launch

# outside network
roslaunch mir_control mir_control.launch mir_hostname:=<MiR100_IP>
```  

### Services
#### send_to_goal
Send robot to a goal saved in `/config/target_goals.yaml`.

#### dock_to_vl_marker
Start docking to the selected marker.

#### get_markers
Get list of saved markers in `/config/markers.yaml`.

#### change_marker_offsets

#### save_mobile_goal

####

### Launch files 
#### `mir_control.launch`
Launches `mir_driver` and the `mir_control_node`.  


- **`mir_namespace:`** sets the namespace of the robot
- **`mir_tf_prefix:`** sets the prefix for all of MiR100 TF frames
- **`mir_ip:`**  tells the robot IP to the driver. The default value is `mir_ip=192.168.12.20` which is the IP of the robot if you are connected to its hotspot. If you are connected over an outside network this argument need to be changed to the `MiR100_IP`
- **`node_start_delay:`** `mir_driver` takes a while to establish a connection to MiR100. Before starting `mir_control_node` this connection need to be established. This sets the delay between starting `mir_driver` and `mir_control_node`


### Nodes 
#### `mir_control_node.py` 
The main control node for MiR100. Initializes a `MiR100` python object. See `mir100_class.py` script for details on the object.

