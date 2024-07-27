# Starting UR5e & Robotiq 2F-85

## Pre-check
Before turning on the power, make sure that:

- The `control computer` is connected to the UR5e controller with an Ethernet cable
- UR5e controller is using the memory stick with the correct configuration. This is important because URCaps conflicts can impede gripper control. The only URCaps on the controller should be the ***externalcontrol-x.x.x.urcap*** and ***rs485-x.x.x.urcap*** URCaps.

## Power
To turn on UR5e press the power button on the teach pendant. Gripper gets its power from the robot manipulator.

## UR5e settings
Go to `Settings -> System -> Network` and configure the network settings. This is the `UR5e_IP` address that the `control computer` uses to control the arm.

```
# example configuration
IP address: 192.168.77.245
Subnet mask: 255.255.255.0
```

Go to `Installation -> URCaps -> External control` and configure the network settings for external control. This is the `control computer` IP address that `UR5e` uses to know where the control is coming from.

```
# example configuration
Host IP: 192.168.77.240
Custom port: 50002
Host name: 192.168.77.240
```