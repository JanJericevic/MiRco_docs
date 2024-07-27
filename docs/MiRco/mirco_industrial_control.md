# MiRco industrial interface control
Before controlling `MiRco` it is recommended to read the [MiRco interface](./mirco_interface.md) chapter of the documentation.

Make sure to follow the **industrial interface** parts of the [Starting `MiRco`](./mirco_start.md) instructions.

- Design the UR program normally.

- Design the MiR mission. In the place where you want to perform robot manipulation insert the `UR -> Run UR program` action. Set the `program name` parameter to the UR program absolute path without the .urp extension. `MiR100` will wait for `UR5e` to finish program execution and then continue with its mission.

<div >
<img src="../img/industrial_mission.png" alt="MiRco industrial interface mission"/>
</div>

## Usage
- set `UR5e` to `Remote control` mode
- add MiR mission to the mission queue
- start mission execution on the web interface

!!! note
    By installing a plugin for the Polyscope software package - a [URCap](https://www.universal-robots.com/plus/products/universal-robots/mir-ur-synchronisation-urcap/), the functionality of the UR teach pendant can be extended. The plugin allows starting and stopping `MiR100` missions, reading and writing `MiR100` registers and accessing `MiR100` states such as battery status, robot status and mission status. This documentation does not cover the use of this URCap.