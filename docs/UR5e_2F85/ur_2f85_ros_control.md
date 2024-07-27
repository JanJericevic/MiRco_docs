# ROS control
This page describes the ROS control for the **UR5e manipulator & Robotiq 2f85 gripper only** . To control the manipulator and gripper as part of the MiRco mobile manipulator see [MiRco ROS control](../MiRco/mirco_ros_control.md).  

Before controlling the robot follow the [Starting UR5e & Robotiq 2F-85](./ur_2f85_start.md) instructions.

## Packages overview
- **`ur5e_2f85:`** main control package
- **`ur5e_2f85_description:`** URDF description, kinematics calibration and controllers configuration files for UR5e with the attached 2f85
- **`ur5e_2f85_moveit_config:`** MoveIt! configuration and launch files
- **`robotiq_2f85:`** Robotiq ROS driver, specifically for 2f85 gripper. Fork of the [original driver](https://github.com/TAMS-Group/robotiq) 

## Usage
TODO: test

## ur5e_2f85 package
The main control package

### Launch files
TODO: test

### Nodes
TODO: test

### Config files
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
#### save_arm_pose
Saves the current manipulator pose as joint states. The pose is saved in `/config/ur5e_save_poses.yaml` under a selected name.

If `/config/ur5e_save_poses.yaml` does not include a pose with the same name, a pose with the selected name is created and saved to the file. If a pose with the same name already exists, it's overwritten with the current joint states.

#### set_arm_pose
Sets manipulator to a selected pose saved as joint states in `/config/ur5e_save_poses.yaml`.  
Uses MoveJ.

#### get_arm_pose
Gets a manipulator pose as joint states. Returns a pose saved in `/config/ur5e_save_poses.yaml` specified with the pose name.

#### get_arm_pose_names
Gets a list of poses saved in `/config/ur5e_save_poses.yaml`.