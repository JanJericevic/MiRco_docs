# Starting MiRco

## Pre-checks
Before turning on the power, make sure that:

- `MiRco` battery has enough charge. If not, you can plug in the cable for [direct power](#direct-power). You can check battery status with the button on the charge controller on `MiRco`'s right side. 
- `MiR100` battery has enough charge. Check by plugging in the charger. LED lights on the charger will indicate battery status. [See MiR charging instructions](../MiR100/mir_start.md/#mir100-charging-instructions).
- Connection with `UR5e` controller:
    - For [`MiRco` industrial interface control](./mirco_interface.md/#mirco-industrial-interface): connect the **red** Ethernet cable to the `UR5e` controller
    - For [`MiRco` ROS interface control](./mirco_interface.md/#mirco-ros-interface): connect the **white** Ethernet cable to the `UR5e` controller
- `UR5e` controller is using the correct configuration. This is important because URCaps conflicts can impede gripper control. The only URCaps on the controller should be the ***externalcontrol-x.x.x.urcap*** and ***rs485-x.x.x.urcap***.

<!-- TODO: add pictures -->

## Power
- turn on the `MiRco` battery: power switch on the left side of the housing. Wait for the double audio signal
- turn on `MiRco PC`: PC power switch inside the housing
- turn on `MiR100`: [blue button](../MiR100/mir_start.md/#power) on the front right corner
- turn on `UR5e`: power button on the teach pendant

## MiR100 settings
!!! warning
    MiR100 internal clock is prone to desynchronization which can mess with the robot control.  
    Before controlling the robot, you have to connect to the [MiR web interface](../MiR100/mir_connection.md/#establish-mir-web-interface-connection) and go to `System -> Settings -> Date & Time` and synchronize the internal clock. Click `Load from device` and `Save changes`.

Get the `MiR100_IP`. In the [MiR web interface](../MiR100/mir_connection.md/#establish-mir-web-interface-connection) go to `System -> Settings -> WiFi`. `MiR100_IP` is listed under the connection details

### MiRco industrial interface
For industrial interface control we need to enter `UR5e_IP` address in the MiR web interface.

- Go to `System -> Settings -> Features` and make sure the *Universal Robots Interface* is enabled.
- Go to `System -> Settings -> UR Interface` and enter `UR5e_IP` address. This IP has to match the IP in the [`UR5e` network settings](#ur5e-settings) bellow.
```
# Universal Robot IP address:
192.168.12.244 # example IP
```

## UR5e settings

### MiRco industrial interface
The IP address of the internal computer in `MiR100` is `192.168.12.20`. To establish a connection between the robots, `UR5e` static IP address should be in the range of `192.168.12.xxx` where xxx is a number from 100–255.

Go to `Settings -> System -> Network` and make sure the network settings are correct.

```
IP address: 192.168.12.244 # example IP
Subnet mask: 255.255.255.0
```

### MiRco ROS interface
Go to `Settings -> System -> Network` and make sure the network settings are correct. This is the `UR5e_IP` address that the `MiRco PC` uses to control the arm.

```
IP address: 192.168.77.245
Subnet mask: 255.255.255.0
```

Go to `Installation -> URCaps -> External control` and make sure the network settings are correct. This is the `MiRco_IP` address that `UR5e` uses to know where the control is coming from.

```
Host IP: 192.168.77.240
Custom port: 50002
Host name: 192.168.77.240
```

#### External control UR program
Design a UR program to use the ***externalcontrol-x.x.x.urcap*** URCap to allow external communication with the manipulator. 

Create a new program and insert the External Control program node into the program tree. Save the program. You will see the `Host IP` and `Custom port` that you configured in the previous step.

There is a program already prepared. To use it load the `home/magisterij/janjericevic/mir100_ur5e_ros` program. 

<!-- TODO: rename ur program, screenshot with correct ip -->

<div >
<img src="../img/ur_external_control.png" alt="UR external control urcap program"/>
</div>


## MiRco charging instructions
### Direct power
`MiRco` can run on direct power from an outlet. This is useful when developing parts of applications where `MiRco` is stationary. This does not power `MiR100`.

Plug the power cable to the direct power plug on the left side of `MiRco`'s housing, below the power switch.

### Start charging
- `MiR100` charging: see [`MiR100` charging instructions](../MiR100/mir_start.md/#mir100-charging-instructions)
- `MiRco` battery: plug in the charger to an outlet. When the charger LED light lights up, plug the charger to the `MiRco` battery charging plug on the left side of `MiRco`'s housing, below the power switch

### Stop charging
- `MiR100`: see [`MiR100` charging instructions](../MiR100/mir_start.md/#mir100-charging-instructions)
- `MiRco` battery: remove the charger from the `MiRco` battery charging plug

<!-- TODO:photos -->