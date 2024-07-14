# MiRco control

**Before controlling `MiRco` read the [MiRco system interface](./mirco_interface.md) chapter of the documentation.**

## 1. MiRco ROS interface

### 1.1 Bringup
**Prepare the robot arm.** On the teach pendant load the `home/magisterij/janjericevic/mir100_ur5e_ros` program. Turn on and start the robot arm.

**Run the `MiRco` bringup launch file.** Wait until the subscribing/publishing messages stop appearing. This means the connection is esablished

```bash
roslaunch mirco_robot mirco_bringup.launch
```

**Start the UR program on the teach pendant.** In the terminal where you launched the bringup file you should see a `Robot connected to reverse interface. Ready to receive control commands` message.

**Run the `MiRco` control launch file**. 
```bash
roslaunch mirco_robot mirco_control.launch
```


<!-- ### 1. MiRco launch files

#### 1.1 mirco_bringup.launch
- Establishes connection to the `MiR100`, `UR5e` and `Robotiq 2F-85`.
- Loads the unified URDF robot model
- Sets the `MiR100_IP` with the `robot_base_ip` argument.
    
    **This should match the IP in the MiR web interface!**

- Sets the `UR5e_IP` with the `ur5e_ip` argument. 

    **This should match the IP in the UR5e interface!**

- Sets the robot namespace with the `mir_namespace` and `ur5e_namespace` arguments.
- Sets the TF frames prefixes with the `mir_prefix`, `ur5e_prefix` and `gripper_prefix` arguments.

#### 1.2 mirco_control.launch
- Starts the `mir_control_node`. This is the main node providing control and services of `MiR100` and establishing connection to the API
- Starts MoveIt  -->


