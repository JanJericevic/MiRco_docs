# MiRco overview

<div class="image">
<img src="../img/MiRco_model.png" class="image_ver" alt="MiRco robot"/>
</div>

`MiRco` is a mobile manipulator consisting of commercially available components combined in a custom housing. It integrates `MiR100` autonomous mobile robot with the `UR5e` manipulator arm. 

Control is possible with both the industrial and ROS interfaces - see [MiRco interfaces](mirco_interface.md).

**MiRco components:**

- **MiR100:** autonomous mobile robot
- **UR5e:** collaborative industrial manipulator
- **Robotiq 2f85:** electronic gripper
- **MiRco PC:** control computer
- **Power:** 24V LiFePO4 battery + 24V DC / 230V AC inverter. Used to power the `UR5e` manipulator. Can also use direct power from an outlet.