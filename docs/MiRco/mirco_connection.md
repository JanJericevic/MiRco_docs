# Connect to MiRco

## 1. Start MiRco
TODO: add pictures
### 1.1 Pre-checks
Before turning on the power, make sure that:

- MiRco battery has enough charge. If not, you can plug in the cable for direct power. You can check battery status with the button on the charge controller. 
- MiR100 battery has enough charge. Check by plugging in the charger. LED lights on the charger will indicate battery status. [See MiR charging instructions (TODO)](TODO).
- The white Ethernet cable from the MiRco computer is connected to the UR5e controller.
- UR5e controller is using the right memory stick. This is important because URCaps conflicts can impede gripper control.  

### 1.2 Power
- turn on the `MiRco` battery switch. Wait for the double audio signal
- turn on `MiRco` computer
- turn on `MiR100`: blue button on the front right corner
- turn on `UR5e` on the teach pendant

### 1.3 UR5e settings
Go to `Settings -> System -> Network` and make sure the network settings are correct. This is the `UR5e` IP address that the `MiRco` computer uses to control the arm.

```bash
IP address: 192.168.77.245
Subnet mask: 255.255.255.0
```

Go to `Installation -> URCaps -> External control` and make sure the network settings are correct. This is the `MiRco` computer IP address so that `UR5e` knows where the control is comming from.

```bash
Host IP: 192.168.77.240
Custom port: 50002
Host name: 192.168.77.240s
```

### 1.4 MiR100 settings
`MiR100` internal clock is prone to desynchronization. [Connect](../MiR100/mir_connection.md) to the MiR web interface and go to `System -> Settings -> Date & Time` and synchronize the internal clock. Load from device and save changes.

## 2. Connecting to MiRco
### 2.1 Connect to the MiRco computer
`MirCo` is connected to the lab network however it does not have a static `MiRco_IP` (for now).

#### 2.1.1 If you know `MiRco_IP`
- **Ssh**: `ssh student@MiRco_IP` and sign in.
- **Remote desktop**: Use NoMachine. When turning on the MiRco computer/your instance of NoMachine, it can take a couple of minutes for MiRco computer to show up under network devices. If you know `MiRco_IP` you don't need to wait. See [this](#22-enabling-and-disabling-headless-remote-desktop-on-the-mirco-computer) about enabling/disabling remote desktop.
- **Connect a display, computer and mouse directly to the computer**. See [this](#22-enabling-and-disabling-headless-remote-desktop-on-the-mirco-computer) about enabling/disabling remote desktop. 

#### 2.1.1 If you don't know `MiRco_IP`
Connect to the computer over remote desktop or plug in a display. When connected, open the terminal and look up `MiRco_IP` with `ifconfig`.

#### 2.2.2 Troubleshooting
If you can't connect remotely to `MiRco`, but the robot is in [headless remote desktop mode](#22-enabling-and-disabling-headless-remote-desktop-on-the-mirco-computer):

- plugin in a display and keyboard
- restart the computer
- boot into a root terminal. When the boot menu comes up: Advanced options for `Ubuntu -> Ubuntu (recovery mode) -> root`
- delete file `/etc/X11/xorg.conf`
- restart the computer

This disables the headless remote desktop mode. From here you can lookup `MiRco_IP` and proceed accordingly.

### 2.2 Enabling and disabling headless remote desktop on the MiRco computer
Remote desktop does not work if no monitor devices are plugged in the computer. 
A workaround are two scripts located in `~/.local/bin`.
This does not affect `ssh` connections.

- `display_headless.sh`: creates a X11 config file that enables headless remote desktop. In headless mode there is no display output even if a display is connected to the computer. **A logout is required.**
- `display_monitor.sh`: removes the X11 config file. You can connect a display but if you remove it, remote desktop will not work.

```bash
#enable headless remote desktop
~/.local/bin/display_headless.sh

#disable headless remote desktop
~/.local/bin/display_monitor.sh
```

## 3. Development tips
- Connection over remote desktop will always be lacking. Use it only when visual tools like `RViz` are needed. When possible connect to the robot over `ssh`. 
- When writing or testing code on MiRco, use the `Remote-SSH` extension for `VSCodium/VSCode`. It enables you to remotely access files on the MiRco computer within your development environment. There is no lag issue like with remote desktop and the robot is still free to move (as opposed to connecting a monitor to MiRco).
- If you are using a Docker installation on your robot, use the `Dev Containers` extension for `VSCode`. It enables you to access container files within your development environment. If you use it in combination with the `Remote-SSH` extension, you can access files within a container running on your robot computer. This extension is only available for `VSCode` and not for `VSCodium`. 