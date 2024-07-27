# Connect to MiR100
There are two ways of interacting with the MiR100 mobile platform:

- [MiR web interface](#establish-mir-web-interface-connection)
- [ROS](#establish-ros-connection)


## Establish MiR web interface connection
You can connect to the web interface over its hotspot or the [outside network](#connect-mir100-to-a-wifi-network) it is connected to.

### Hotspot
- connect to the MiR_R*** hotspot with the password
- open mir.com (default IP = 192.168.12.20) in your browser
- use your credentials to log in to the web interface

### Outside network
- you have to know the `MiR100_IP`. If you don't, connect to the web interface over hotspot and go to `System -> Settings -> WiFi`. `MiR100_IP` is listed under the connection details
- open `MiR100_IP` in your browser
- use your credentials to log in to the web interface

!!! warning
    MiR100 internal clock is prone to desynchronization which can mess with the robot control.  
    First thing you have to do after you connect to the MiR web interface is go to `System -> Settings -> Date & Time` and synchronize the internal clock. Click `Load from device` and `Save changes`.



## Establish ROS connection
MiR100 has a running ROS instance on its internal computer. This means you can:

- connect to its internal `roscore`
- **recommended**: connect using the `mir_control` package

### Internal roscore
**You have to be connected to the robot hotspot (MiR_R\*\*\*\*).**  
Set the ROS master addres to the address of MiR100 internal roscore and test the connection.

```bash
# this needs to be done in every terminal
export ROS_MASTER_URI=http://192.168.12.20:11311
export ROS_HOSTNAME=<your IP: 192.168.12.***>

# test the connection
# test subscriber
rostopic list
rostopic echo /odom

# test publisher
rostopic pub -r 50 /cmd_vel geometry_msgs/TwistStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.2"
```

### mir_control package
Using `mir_control` you can establish a ROS connection over the MiR100 hotspot or the [outside network](#connect-mir100-to-a-wifi-network) MiR100 is connected to.

#### Hotspot
If you are connected to the MiR_R**** hotspot:

```bash
# launch mir_control
roslaunch mir_control mir_control.launch
```

Terminal output will hang for a bit on "ROS bridge connected" message.  
Wait until "MiR100: ..." messages finish, then continue.

```bash
# test the connection
# test subscriber
rostopic echo /odom

# test publisher
rostopic pub -r 50 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

#### Outside network
If you are connected to the same outside network as MiR100:

```bash
# launch mir_control
roslaunch mir_control mir_control.launch mir_ip:=<MiR100_IP>
```
Terminal output will hang for a bit on "ROS bridge connected" message.  
Wait until "MiR100: ..." messages finish, then continue.

```bash
# test the connection
# test subscriber
rostopic echo /mir100/odom

# test publisher
rostopic pub -r 50 /mir100/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

!!! note
    
    When using `mir_control` we send `geometry_msgs/Twist` type messages instead of `geometry_msgs/TwistStamped` to the `/cmd_vel` topic. That is because the `mir_driver` package expects messages of type `geometry_msgs/Twist` on the `/cmd_vel` topic and converts them to `geometry_msgs/TwistStamped` messages before sending the commands to the robot.

## Connect MiR100 to a WiFi network
You can connect the robot to an outside network:

- connect to the MiR_R**** hotspot
- in the web interface go to `System -> Settings -> WiFi`
- select "Add connection"
- select the network and fill in required information
- when you're finished select "Add connection"
- `MiR100_IP` is displayed under the network connection details. You can use this IP to access the web interface or when using `mir_control`
