# File structure
This page describes the file structure on the `MiRco` computer. It describes the core files that enable `MiRco` ROS control.

Files are organized in a ROS `catkin` workspace. If you are not familiar, [ROS concepts](http://wiki.ros.org/ROS/Concepts) and [catkin workspaces](http://wiki.ros.org/catkin/workspaces) of the official ROS documentation are a good place to start. For a new project/application create a package within the `src` directory or create a new workspace and source both the new and `MiRco` workspaces.

```bash
MiRco_ws/src
├── CMakeLists.txt                  # main CMakeLists file
├── MiRco_robot                     
│   ├── convenience_scripts         # TODO delete
│   ├── Dockerfile                  # Docker file, see installation for info
│   ├── mir_analysis                # TODO delete   
│   ├── mirco_description           # MiRco unified URDF
│   ├── mirco_moveit_config         # MiRco MoveIt config
│   ├── mir_control                 # MiR100 control node, services, driver launch
│   ├── mirco_robot                 # main launch files, main app
│   ├── mir_joy_teleop              # MiR100 joystick control
│   ├── mir_rest_api                # MiR100 REST API package
│   ├── README.md
│   ├── ur5e_2f85                   # Arm & gripper control node, launch files, services
│   ├── ur5e_2f85_description       # Arm & gripper URDF
│   └── ur5e_2f85_moveit_config     # Arm & gripper MoveIt config
├── mir_robot                       # MiR100 ROS driver
│   ├── Dockerfile-noetic
│   ├── LICENSE
│   ├── mir_actions
│   ├── mir_description
│   ├── mir_driver
│   ├── mir_dwb_critics
│   ├── mir_gazebo
│   ├── mir_msgs
│   ├── mir_navigation
│   ├── mir_robot
│   ├── README.md
│   └── sdc21x0
├── robotiq_2f85                    # gripper ROS driver 
    ├── CONTRIBUTING.md
    ├── LICENSE
    ├── README.md
    ├── robotiq
    ├── robotiq_2f_85_gripper_visualization
    ├── robotiq_2f_gripper_action_server
    ├── robotiq_2f_gripper_control
    ├── robotiq_ethercat
    ├── robotiq_modbus_rtu
    └── robotiq_modbus_tcp
```